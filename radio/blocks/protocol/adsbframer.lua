---
-- Validate and extract ADS-B frames from a bit stream.
--
-- Each frame is checked against the 24 bit Mode S parity field: the syndrome
-- of a valid frame is zero. Frames with a non-zero syndrome are discarded.
--
-- DF11 all-call replies, and DF17/DF18 extended squitters, are decoded. The
-- extended squitter payload is decoded according to its type code: aircraft
-- identification, surface position, airborne position, and airborne velocity
-- are supported.
--
-- Aircraft positions are broadcast as alternating even and odd compact
-- position reports, which are only unambiguous as a pair. The framer keeps
-- per-aircraft state to pair them, and falls back to locally unambiguous
-- decoding against the aircraft's last known position. Surface positions
-- additionally require a receiver reference position, since they only encode
-- the low order bits of a 90 degree zone.
--
-- Setting `ADSBFramerBlock.ADSBFrameType.color` colours the text representation
-- of a frame by aircraft, for readability when several aircraft are interleaved
-- in a terminal. It is off by default, and does not affect the JSON
-- representation or the decoded field values.
--
-- @category Protocol
-- @block ADSBFramerBlock
-- @tparam[opt={}] table options Additional options, specifying:
--                               * `symbol_rate` (number, symbol rate of the
--                                 incoming bit stream, in symbols per second,
--                                 default 1e6)
--                               * `reference` (table, receiver position with
--                                 `lat` and `lon` keys, required for surface
--                                 positions)
--
-- @signature in:Bit > out:ADSBFrameType
--
-- @usage
-- local framer = radio.ADSBFramerBlock({reference = {lat = 51.99, lon = 4.375}})

---
-- ADS-B frame type, a Lua object with properties:
--
-- ``` text
-- {
--   df = <5-bit integer>,
--   icao = <24-bit integer>,
--   hex = <string>,
--   type_code = <5-bit integer>,
--   subtype = <3-bit integer>,
--   callsign = <string>,
--   category = <3-bit integer>,
--   emergency_state = <3-bit integer>,
--   altitude = <integer, feet>,
--   surveillance_status = <2-bit integer>,
--   cpr_format = <1-bit integer>,
--   cpr_lat = <17-bit integer>,
--   cpr_lon = <17-bit integer>,
--   latitude = <number, degrees>,
--   longitude = <number, degrees>,
--   groundspeed = <number, knots>,
--   track = <number, degrees>,
--   vertical_rate = <integer, feet per minute>,
--   vertical_rate_source = <string>,
--   gnss_baro_diff = <integer, feet>,
--   airspeed = <number, knots>,
--   airspeed_type = <string>,
--   heading = <number, degrees>,
-- }
-- ```
--
-- Only `df` and `icao` are always present. The remaining properties are set
-- according to the downlink format and type code of the frame, and are
-- absent when they do not apply.
--
-- @datatype ADSBFramerBlock.ADSBFrameType
-- @tparam int df Downlink format field, 5-bits wide
-- @tparam int icao ICAO address field, 24-bits wide
-- @tparam string hex Raw frame, as hex digits
-- @tparam int type_code ADS-B type code field, 5-bits wide
-- @tparam int subtype ADS-B velocity subtype field, 3-bits wide
-- @tparam string callsign Aircraft identification
-- @tparam int category Emitter category field, 3-bits wide
-- @tparam int emergency_state Emergency state field, 3-bits wide
-- @tparam int altitude Barometric or GNSS altitude, in feet
-- @tparam int surveillance_status Surveillance status field, 2-bits wide
-- @tparam int cpr_format Compact position report format, 0 even and 1 odd
-- @tparam int cpr_lat Encoded compact position report latitude, 17-bits wide
-- @tparam int cpr_lon Encoded compact position report longitude, 17-bits wide
-- @tparam number latitude Decoded latitude, in degrees
-- @tparam number longitude Decoded longitude, in degrees
-- @tparam number groundspeed Decoded ground speed, in knots
-- @tparam number track Decoded ground track, in degrees
-- @tparam int vertical_rate Decoded vertical rate, in feet per minute
-- @tparam string vertical_rate_source Vertical rate source, "gnss" or "baro"
-- @tparam int gnss_baro_diff Difference between the GNSS and barometric
--                            altitudes, in feet
-- @tparam number airspeed Decoded airspeed, in knots
-- @tparam string airspeed_type Airspeed type, "ias" or "tas"
-- @tparam number heading Decoded heading, in degrees

local bit = require('bit')

local block = require('radio.core.block')
local types = require('radio.types')

-- Downlink formats
local DF_ALL_CALL_REPLY = 11
local DF_EXTENDED_SQUITTER = 17
local DF_EXTENDED_SQUITTER_TISB = 18

-- Frame lengths, in bits
local SHORT_FRAME_BITS = 56
local LONG_FRAME_BITS = 112

-- Mode S generator polynomial, x^24 + ... + x^3 + 1
local CRC_POLY = 0x1FFF409

-- Number of latitude zones between the equator and a pole
local NZ = 15

-- Compact position report scale, 2^17
local CPR_DENOM = 131072.0

-- Maximum age difference between an even and odd position report pair, in
-- seconds, and maximum age of a track before it is forgotten
local POSITION_PAIR_MAX_AGE = 10
local TRACK_MAX_AGE = 900

-- Maximum number of aircraft tracked before stale entries are dropped
local TRACK_MAX_AIRCRAFT = 4096

-- Callsign character table: 6-bit ICAO alphabet
local CALLSIGN_TABLE = {}
for i = 0, 63 do
    if i >= 1 and i <= 26 then
        CALLSIGN_TABLE[i] = string.char(i + 64)
    elseif i == 32 or (i >= 48 and i <= 57) then
        CALLSIGN_TABLE[i] = string.char(i)
    else
        CALLSIGN_TABLE[i] = "#"
    end
end

-- Surface position movement (ground speed) encoding bins
local MOVEMENT_LIMITS = {2, 9, 13, 39, 94, 109, 124}
local MOVEMENT_SPEEDS = {0.125, 1, 2, 15, 70, 100, 175}
local MOVEMENT_STEPS = {0.125, 0.25, 0.5, 1, 2, 5}

-- Framing helpers

local function adsb_modulo(x, y)
    return x - y * math.floor(x / y)
end

-- Gillham (reflected Gray) code to integer
local function adsb_gray_to_integer(x)
    x = bit.bxor(x, bit.rshift(x, 8))
    x = bit.bxor(x, bit.rshift(x, 4))
    x = bit.bxor(x, bit.rshift(x, 2))
    x = bit.bxor(x, bit.rshift(x, 1))

    return x
end

-- Decode a 13 bit altitude code field to feet, returning nil for invalid or
-- unavailable codes. Q=1 codes are a linear 25 foot encoding, Q=0 codes are
-- the legacy 100 foot Gillham encoding.
local function adsb_decode_altitude_code(ac)
    if ac == 0 then
        return nil
    end

    local m_bit = bit.band(bit.rshift(ac, 6), 1)
    local q_bit = bit.band(bit.rshift(ac, 4), 1)

    if m_bit == 0 and q_bit == 1 then
        -- 25 foot interval: drop M and Q, the remaining 11 bits form N
        local n = bit.bor(bit.bor(bit.band(bit.rshift(ac, 2), 0x7E0), bit.band(bit.rshift(ac, 1), 0x10)), bit.band(ac, 0xF))

        return n * 25 - 1000
    end

    if m_bit == 0 and q_bit == 0 then
        -- 100 foot interval: a 500 foot counter and a 100 foot counter
        local function bit_at(position)
            return bit.band(bit.rshift(ac, 12 - position), 1)
        end

        local c1, a1 = bit_at(0), bit_at(1)
        local c2, a2 = bit_at(2), bit_at(3)
        local c4, a4 = bit_at(4), bit_at(5)
        local b1 = bit_at(7)
        local b2, d2 = bit_at(9), bit_at(10)
        local b4, d4 = bit_at(11), bit_at(12)

        local gc500 = bit.bor(bit.bor(bit.bor(bit.bor(bit.bor(bit.bor(bit.bor(
            bit.lshift(d2, 7), bit.lshift(d4, 6)), bit.lshift(a1, 5)),
            bit.lshift(a2, 4)), bit.lshift(a4, 3)), bit.lshift(b1, 2)),
            bit.lshift(b2, 1)), b4)
        local gc100 = bit.bor(bit.lshift(c1, 2), bit.bor(bit.lshift(c2, 1), c4))

        local n500 = adsb_gray_to_integer(gc500)
        local n100 = adsb_gray_to_integer(gc100)

        -- 100 foot counters of 0, 5, and 6 are invalid, 7 is remapped to 5
        if n100 == 0 or n100 == 5 or n100 == 6 then
            return nil
        end
        if n100 == 7 then
            n100 = 5
        end

        -- An odd 500 foot counter inverts the direction of the 100 foot counter
        if n500 % 2 == 1 then
            n100 = 6 - n100
        end

        return n500 * 500 + n100 * 100 - 1300
    end

    -- M=1 is a rare metric encoding
    return nil
end

-- Decode the number of longitude zones at a latitude
local function adsb_nl(lat)
    local a = math.abs(lat)

    if a > 87.0 then
        return 1
    elseif a == 87.0 then
        return 2
    end

    local x = 1 - math.cos(math.pi / (2 * NZ))
    local y = math.cos(math.pi / 180 * a) ^ 2
    local nl = math.floor(2 * math.pi / math.acos(1 - x / y))

    if nl > 59 then
        nl = 59
    elseif nl < 1 then
        nl = 1
    end

    return nl
end

-- Globally unambiguous position from an even/odd compact position report pair
local function adsb_airborne_position(lat_even, lon_even, lat_odd, lon_odd, even_is_newer)
    local cprlat_even, cprlon_even = lat_even / CPR_DENOM, lon_even / CPR_DENOM
    local cprlat_odd, cprlon_odd = lat_odd / CPR_DENOM, lon_odd / CPR_DENOM

    local j = math.floor(59 * cprlat_even - 60 * cprlat_odd + 0.5)

    local rlat_even = (360.0 / 60) * (adsb_modulo(j, 60) + cprlat_even)
    local rlat_odd = (360.0 / 59) * (adsb_modulo(j, 59) + cprlat_odd)

    if rlat_even >= 270 then
        rlat_even = rlat_even - 360
    end
    if rlat_odd >= 270 then
        rlat_odd = rlat_odd - 360
    end

    if adsb_nl(rlat_even) ~= adsb_nl(rlat_odd) then
        return nil
    end

    local lat, lon

    if even_is_newer then
        lat = rlat_even
        local nl = adsb_nl(lat)
        local ni = math.max(nl, 1)
        local m = math.floor(cprlon_even * (nl - 1) - cprlon_odd * nl + 0.5)
        lon = (360.0 / ni) * (adsb_modulo(m, ni) + cprlon_even)
    else
        lat = rlat_odd
        local nl = adsb_nl(lat)
        local ni = math.max(nl - 1, 1)
        local m = math.floor(cprlon_even * (nl - 1) - cprlon_odd * nl + 0.5)
        lon = (360.0 / ni) * (adsb_modulo(m, ni) + cprlon_odd)
    end

    if lon > 180 then
        lon = lon - 360
    end

    -- A latitude outside the poles means the pair straddled a zone boundary
    if math.abs(lat) > 90 or math.abs(lon) > 180 then
        return nil
    end

    return lat, lon
end

-- Locally unambiguous airborne position, from a single report and a nearby
-- reference position
local function adsb_airborne_position_with_reference(cpr_format, lat_raw, lon_raw, lat_ref, lon_ref)
    local cpr_lat, cpr_lon = lat_raw / CPR_DENOM, lon_raw / CPR_DENOM

    local dlat = (cpr_format == 0) and (360.0 / 60) or (360.0 / 59)
    local j = math.floor(0.5 + lat_ref / dlat - cpr_lat)
    local lat = dlat * (j + cpr_lat)

    local ni = adsb_nl(lat) - cpr_format
    local dlon = (ni > 0) and (360.0 / ni) or 360.0
    local m = math.floor(0.5 + lon_ref / dlon - cpr_lon)
    local lon = dlon * (m + cpr_lon)

    return lat, lon
end

-- Locally unambiguous surface position, from a single report and a nearby
-- reference position
local function adsb_surface_position_with_reference(cpr_format, lat_raw, lon_raw, lat_ref, lon_ref)
    local cpr_lat, cpr_lon = lat_raw / CPR_DENOM, lon_raw / CPR_DENOM

    local dlat = (cpr_format == 0) and (90.0 / 60) or (90.0 / 59)
    local j = math.floor(0.5 + lat_ref / dlat - cpr_lat)
    local lat = dlat * (j + cpr_lat)

    local ni = adsb_nl(lat) - cpr_format
    local dlon = (ni > 0) and (90.0 / ni) or 90.0
    local m = math.floor(0.5 + lon_ref / dlon - cpr_lon)
    local lon = dlon * (m + cpr_lon)

    return lat, lon
end

-- Surface position from an even/odd pair, resolved against a reference
-- position because surface reports only encode a 90 degree zone
local function adsb_surface_position(lat_even, lon_even, lat_odd, lon_odd, lat_ref, lon_ref, even_is_newer)
    local cprlat_even, cprlon_even = lat_even / CPR_DENOM, lon_even / CPR_DENOM
    local cprlat_odd, cprlon_odd = lat_odd / CPR_DENOM, lon_odd / CPR_DENOM

    local j = math.floor(59 * cprlat_even - 60 * cprlat_odd + 0.5)

    local rlat_even_n = (90.0 / 60) * (adsb_modulo(j, 60) + cprlat_even)
    local rlat_odd_n = (90.0 / 59) * (adsb_modulo(j, 59) + cprlat_odd)

    -- Two candidate 90 degree latitude zones, resolved by distance to the
    -- reference rather than by hemisphere
    local rlat_even, rlat_odd

    if math.abs(lat_ref - (rlat_even_n - 90)) < math.abs(lat_ref - rlat_even_n) then
        rlat_even, rlat_odd = rlat_even_n - 90, rlat_odd_n - 90
    else
        rlat_even, rlat_odd = rlat_even_n, rlat_odd_n
    end

    if adsb_nl(rlat_even) ~= adsb_nl(rlat_odd) then
        return nil
    end

    local lat, lon_base

    if even_is_newer then
        lat = rlat_even
        local nl = adsb_nl(lat)
        local ni = math.max(nl, 1)
        local m = math.floor(cprlon_even * (nl - 1) - cprlon_odd * nl + 0.5)
        lon_base = (90.0 / ni) * (adsb_modulo(m, ni) + cprlon_even)
    else
        lat = rlat_odd
        local nl = adsb_nl(lat)
        local ni = math.max(nl - 1, 1)
        local m = math.floor(cprlon_even * (nl - 1) - cprlon_odd * nl + 0.5)
        lon_base = (90.0 / ni) * (adsb_modulo(m, ni) + cprlon_odd)
    end

    -- Four candidate 90 degree longitude quadrants, resolved by circular
    -- distance to the reference
    local lon, distance

    for _, quadrant in ipairs({0, 90, 180, 270}) do
        local candidate = adsb_modulo(lon_base + quadrant + 180, 360) - 180
        local candidate_distance = math.abs(adsb_modulo(candidate - lon_ref + 180, 360) - 180)

        if distance == nil or candidate_distance < distance then
            lon, distance = candidate, candidate_distance
        end
    end

    return lat, lon
end

-- Decode the 7 bit surface movement field to knots
local function adsb_decode_movement(movement)
    if movement == 0 or movement > 124 then
        return nil
    elseif movement == 1 then
        return 0.0
    elseif movement == 124 then
        return 175.0
    end

    local i
    for k = 1, #MOVEMENT_LIMITS do
        if MOVEMENT_LIMITS[k] > movement then
            i = k
            break
        end
    end

    return MOVEMENT_SPEEDS[i - 1] + (movement - MOVEMENT_LIMITS[i - 1]) * MOVEMENT_STEPS[i - 1]
end

-- ADS-B Frame Type

-- Background colours from the dark corner of the 6x6x6 colour cube, which
-- stay legible under default terminal text
local COLOR_BACKGROUNDS = {}
for r = 0, 2 do
    for g = 0, 2 do
        for b = 0, 2 do
            COLOR_BACKGROUNDS[#COLOR_BACKGROUNDS + 1] = 16 + 36 * r + 6 * g + b
        end
    end
end

local ADSBFrameType = types.ObjectType.factory()

-- Colour the text representation by aircraft, so that aircraft interleaved in
-- a terminal are easier to tell apart. Off by default; an application turns it
-- on when its output is a terminal. The JSON representation is unaffected.
ADSBFrameType.color = false

function ADSBFrameType.new(properties)
    local self = setmetatable({}, ADSBFrameType)

    for k, v in pairs(properties) do
        self[k] = v
    end

    return self
end

function ADSBFrameType:__tostring()
    local fields = {string.format("df=%u", self.df), string.format("icao=0x%06x", self.icao)}

    if self.hex then
        fields[#fields + 1] = string.format("hex=%s", self.hex)
    end
    if self.callsign then
        fields[#fields + 1] = string.format("callsign=%s", self.callsign)
    end
    if self.category then
        fields[#fields + 1] = string.format("category=%u", self.category)
    end
    if self.emergency_state then
        fields[#fields + 1] = string.format("emergency_state=%u", self.emergency_state)
    end
    if self.altitude then
        fields[#fields + 1] = string.format("altitude=%d", self.altitude)
    end
    if self.surveillance_status then
        fields[#fields + 1] = string.format("surveillance_status=%u", self.surveillance_status)
    end
    if self.groundspeed then
        fields[#fields + 1] = string.format("groundspeed=%.1f", self.groundspeed)
    end
    if self.track then
        fields[#fields + 1] = string.format("track=%.1f", self.track)
    end
    if self.vertical_rate then
        fields[#fields + 1] = string.format("vertical_rate=%+d", self.vertical_rate)
    end
    if self.vertical_rate_source then
        fields[#fields + 1] = string.format("vertical_rate_source=%s", self.vertical_rate_source)
    end
    if self.gnss_baro_diff then
        fields[#fields + 1] = string.format("gnss_baro_diff=%+d", self.gnss_baro_diff)
    end
    if self.airspeed then
        fields[#fields + 1] = string.format("airspeed=%.1f", self.airspeed)
    end
    if self.airspeed_type then
        fields[#fields + 1] = string.format("airspeed_type=%s", self.airspeed_type)
    end
    if self.heading then
        fields[#fields + 1] = string.format("heading=%.1f", self.heading)
    end
    if self.latitude and self.longitude then
        fields[#fields + 1] = string.format("latitude=%.5f", self.latitude)
        fields[#fields + 1] = string.format("longitude=%.5f", self.longitude)
    end
    if self.cpr_format then
        fields[#fields + 1] = string.format("cpr_format=%u", self.cpr_format)
    end
    if self.cpr_lat then
        fields[#fields + 1] = string.format("cpr_lat=%u", self.cpr_lat)
        fields[#fields + 1] = string.format("cpr_lon=%u", self.cpr_lon)
    end
    if self.type_code then
        fields[#fields + 1] = string.format("type_code=%u", self.type_code)
    end
    if self.subtype then
        fields[#fields + 1] = string.format("subtype=%u", self.subtype)
    end

    local line = "ADSBFrame<" .. table.concat(fields, ", ") .. ">"

    if ADSBFrameType.color then
        local background = COLOR_BACKGROUNDS[(self.icao % #COLOR_BACKGROUNDS) + 1]

        return string.format("\27[48;5;%dm%s\27[49m", background, line)
    end

    return line
end

-- Compare two frames, with a tolerance on the decoded floating point fields
function ADSBFrameType:approx_equal(other, epsilon)
    epsilon = epsilon or 0.0

    for k, v in pairs(self) do
        local w = other[k]

        if type(v) == "number" and type(w) == "number" then
            if math.abs(v - w) > epsilon then
                return false
            end
        elseif v ~= w then
            return false
        end
    end

    for k in pairs(other) do
        if self[k] == nil then
            return false
        end
    end

    return true
end

-- ADS-B Framer Block

local ADSBFramerBlock = block.factory("ADSBFramerBlock")

ADSBFramerBlock.ADSBFrameType = ADSBFrameType

function ADSBFramerBlock:instantiate(options)
    options = options or {}

    self.symbol_rate = options.symbol_rate or 1e6
    self.reference = options.reference

    self:add_type_signature({block.Input("in", types.Bit)}, {block.Output("out", ADSBFrameType)})
end

function ADSBFramerBlock:initialize()
    -- Frame bit accumulator
    self.frame = types.Bit.vector(LONG_FRAME_BITS)
    self.frame_length = 0
    self.frame_bits = nil

    -- Syndrome scratch space
    self.work = types.Bit.vector(LONG_FRAME_BITS)

    -- Per-aircraft compact position report state
    self.aircraft = {}
    self.aircraft_count = 0

    self.samples = 0

    self.out = ADSBFrameType.vector()
end

-- Compute the Mode S syndrome of a frame: the CRC of the data portion, XORed
-- with the transmitted parity field. Zero for a valid frame. DF11, DF17, and
-- DF18 announce their address in the message body, so no address overlay has
-- to be undone.
function ADSBFramerBlock:syndrome(num_bits)
    local work = self.work

    for i = 0, num_bits-1 do
        work.data[i].value = self.frame.data[i].value
    end

    for i = 0, num_bits-25 do
        if work.data[i].value == 1 then
            for j = 0, 24 do
                work.data[i + j].value = bit.bxor(work.data[i + j].value, bit.band(bit.rshift(CRC_POLY, 24 - j), 1))
            end
        end
    end

    return types.Bit.tonumber(work, num_bits - 24, 24)
end

-- Extract an unsigned integer field from the frame
function ADSBFramerBlock:field(offset, length)
    return types.Bit.tonumber(self.frame, offset, length)
end

-- The raw frame, as lower case hex digits
function ADSBFramerBlock:hex(num_bits)
    local bytes = types.Bit.tobytes(self.frame, 0, num_bits)

    return (bytes:gsub(".", function(c) return string.format("%02x", string.byte(c)) end))
end

-- Extract an 8 character callsign field from the frame
function ADSBFramerBlock:callsign(offset)
    local chars = {}

    for i = 0, 7 do
        chars[i + 1] = CALLSIGN_TABLE[self:field(offset + 6 * i, 6)]
    end

    return (table.concat(chars):gsub("^%s+", ""):gsub("%s+$", ""))
end

-- Decode the ADS-B extended squitter payload (ME field) of a DF17/DF18 frame
function ADSBFramerBlock:squitter(frame)
    local type_code = self:field(32, 5)

    frame.type_code = type_code

    if type_code >= 1 and type_code <= 4 then
        frame.category = self:field(37, 3)
        frame.callsign = self:callsign(40)

    elseif type_code >= 5 and type_code <= 8 then
        frame.groundspeed = adsb_decode_movement(self:field(37, 7))

        if self:field(44, 1) == 1 then
            frame.track = self:field(45, 7) * 360 / 128
        end

        frame.cpr_format = self:field(53, 1)
        frame.cpr_lat = self:field(54, 17)
        frame.cpr_lon = self:field(71, 17)

    elseif (type_code >= 9 and type_code <= 18) or (type_code >= 20 and type_code <= 22) then
        frame.surveillance_status = self:field(37, 2)

        if type_code >= 20 then
            -- GNSS altitude is encoded in metres
            frame.altitude = math.floor(self:field(40, 12) * 3.28084 + 0.5)
        else
            -- Barometric altitude has an M bit inserted at position 6
            local ac = self:field(40, 12)
            local ac13 = bit.bor(bit.lshift(bit.rshift(ac, 6), 7), bit.band(ac, 0x3F))
            frame.altitude = adsb_decode_altitude_code(ac13)
        end

        frame.cpr_format = self:field(53, 1)
        frame.cpr_lat = self:field(54, 17)
        frame.cpr_lon = self:field(71, 17)

    elseif type_code == 19 then
        local subtype = self:field(37, 3)
        frame.subtype = subtype

        frame.vertical_rate_source = (self:field(67, 1) == 0) and "gnss" or "baro"

        -- Vertical rate
        local vertical_rate = self:field(69, 9)

        if vertical_rate ~= 0 then
            frame.vertical_rate = 64 * (vertical_rate - 1)
            if self:field(68, 1) == 1 then
                frame.vertical_rate = -frame.vertical_rate
            end
        end

        -- Difference between the GNSS and barometric altitudes
        local difference = self:field(81, 7)

        if difference ~= 0 then
            frame.gnss_baro_diff = (difference - 1) * 25
            if self:field(80, 1) == 1 then
                frame.gnss_baro_diff = -frame.gnss_baro_diff
            end
        end

        if subtype == 1 or subtype == 2 then
            local scale = (subtype == 2) and 4 or 1

            local velocity_ew = self:field(46, 10)
            local velocity_ns = self:field(57, 10)

            if velocity_ew ~= 0 and velocity_ns ~= 0 then
                local x = (velocity_ew - 1) * scale
                local y = (velocity_ns - 1) * scale

                if self:field(45, 1) == 1 then
                    x = -x
                end
                if self:field(56, 1) == 1 then
                    y = -y
                end

                frame.groundspeed = math.sqrt(x * x + y * y)
                frame.track = adsb_modulo(math.atan2(x, y) * 360 / (2 * math.pi), 360)
            end

        elseif subtype == 3 or subtype == 4 then
            local scale = (subtype == 4) and 4 or 1

            if self:field(45, 1) == 1 then
                frame.heading = self:field(46, 10) * 360 / 1024
            end

            frame.airspeed_type = (self:field(56, 1) == 1) and "tas" or "ias"

            local airspeed = self:field(57, 10)

            if airspeed ~= 0 then
                frame.airspeed = (airspeed - 1) * scale
            end
        end

    elseif type_code == 28 then
        local subtype = self:field(37, 3)
        frame.subtype = subtype

        if subtype == 1 then
            frame.emergency_state = self:field(40, 3)
        end
    end
end

-- Whether a type code carries a compact position report
local function adsb_is_position(type_code)
    return (type_code >= 5 and type_code <= 8) or
           (type_code >= 9 and type_code <= 18) or
           (type_code >= 20 and type_code <= 22)
end

-- Resolve the position of a position frame against the aircraft's own
-- reports, falling back to locally unambiguous decoding against the last
-- known position or the receiver reference position
function ADSBFramerBlock:position(frame)
    local aircraft = self.aircraft[frame.icao]

    if not aircraft then
        aircraft = {}
        self.aircraft[frame.icao] = aircraft
        self.aircraft_count = self.aircraft_count + 1

        -- Keep the tracking table bounded on long runs
        if self.aircraft_count > TRACK_MAX_AIRCRAFT then
            local cutoff = self.samples / self.symbol_rate - TRACK_MAX_AGE

            for icao, entry in pairs(self.aircraft) do
                if entry.seen < cutoff then
                    self.aircraft[icao] = nil
                    self.aircraft_count = self.aircraft_count - 1
                end
            end
        end
    end

    local now = self.samples / self.symbol_rate
    aircraft.seen = now
    aircraft[(frame.cpr_format == 0) and "even" or "odd"] = {lat = frame.cpr_lat, lon = frame.cpr_lon, time = now}

    local surface = frame.type_code <= 8

    local even, odd = aircraft.even, aircraft.odd
    local reference = aircraft.latitude and {lat = aircraft.latitude, lon = aircraft.longitude} or self.reference

    -- Globally unambiguous decoding from an even/odd pair
    if even and odd and math.abs(even.time - odd.time) <= POSITION_PAIR_MAX_AGE then
        local even_is_newer = even.time >= odd.time
        local latitude, longitude

        if surface then
            if reference then
                latitude, longitude = adsb_surface_position(even.lat, even.lon, odd.lat, odd.lon,
                                                            reference.lat, reference.lon, even_is_newer)
            end
        else
            latitude, longitude = adsb_airborne_position(even.lat, even.lon, odd.lat, odd.lon, even_is_newer)
        end

        if latitude then
            frame.latitude, frame.longitude = latitude, longitude
            aircraft.latitude, aircraft.longitude = latitude, longitude
        end
    end

    -- Locally unambiguous decoding against a known position
    if not frame.latitude and reference then
        if surface then
            frame.latitude, frame.longitude = adsb_surface_position_with_reference(frame.cpr_format, frame.cpr_lat,
                                                                                  frame.cpr_lon, reference.lat,
                                                                                  reference.lon)
        else
            frame.latitude, frame.longitude = adsb_airborne_position_with_reference(frame.cpr_format, frame.cpr_lat,
                                                                                    frame.cpr_lon, reference.lat,
                                                                                    reference.lon)
        end
    end
end

-- Validate and decode a complete frame, returning nil if it is not a valid
-- ADS-B frame
function ADSBFramerBlock:decode(num_bits)
    if self:syndrome(num_bits) ~= 0 then
        return nil
    end

    local df = self:field(0, 5)

    local frame = {df = df, icao = self:field(8, 24), hex = self:hex(num_bits)}

    if df == DF_EXTENDED_SQUITTER or df == DF_EXTENDED_SQUITTER_TISB then
        self:squitter(frame)

        if frame.type_code and adsb_is_position(frame.type_code) then
            self:position(frame)
        end

    elseif df ~= DF_ALL_CALL_REPLY then
        return nil
    end

    return ADSBFrameType(frame)
end

function ADSBFramerBlock:process(x)
    local out = self.out:resize(0)

    for i = 0, x.length-1 do
        self.frame.data[self.frame_length] = x.data[i]
        self.frame_length = self.frame_length + 1
        self.samples = self.samples + 1

        -- The downlink format in the first five bits selects the frame length
        if self.frame_length == 5 then
            self.frame_bits = (self:field(0, 5) >= 16) and LONG_FRAME_BITS or SHORT_FRAME_BITS
        end

        if self.frame_bits and self.frame_length == self.frame_bits then
            local frame = self:decode(self.frame_length)

            if frame then
                out:append(frame)
            end

            self.frame_length = 0
            self.frame_bits = nil
        end
    end

    return out
end

return ADSBFramerBlock
