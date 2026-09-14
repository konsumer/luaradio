---
-- Demodulate a 1090 MHz Mode S extended squitter signal into frames.
--
-- The incoming envelope is resampled onto a two sample per symbol grid by
-- linear interpolation, the eight microsecond PPM preamble is detected, and a
-- complete Mode S frame is sliced per detection. The frame length is read from
-- the five bit downlink format, which selects between short (56 bit) and long
-- (112 bit) frames.
--
-- Preamble detection uses only comparisons between samples, so no absolute
-- signal level or gain calibration is assumed: each preamble pulse must stand
-- out from the samples either side of it, the samples between the pulses must
-- be low, and the space between the preamble and the first bit must be low, all
-- relative to the pulse level.
--
-- Frames whose sampling phase falls between samples are recovered by
-- estimating whether the frame is sampled early or late from the energy that
-- leaks into the preamble samples either side of the pulses, rescaling the
-- frame samples by the amount they are expected to be off, and re-slicing.
-- This is the common case at any sample rate that is not an exact multiple of
-- the symbol rate.
--
-- Only frames that pass the 24 bit Mode S parity check are emitted, since the
-- parity check is the only way to choose between the plain and phase corrected
-- demodulation of the same preamble.
--
-- @category Digital
-- @block ADSBDetectorBlock
-- @tparam[opt=1e6] number symbol_rate Symbol rate in symbols per second.
--
-- @signature in:Float32 > out:Bit
--
-- @usage
-- local detector = radio.ADSBDetectorBlock(1e6)

local ffi = require('ffi')
local bit = require('bit')

local block = require('radio.core.block')
local types = require('radio.types')

-- The demodulator works on two samples per symbol, since the Mode S preamble
-- and the bit decisions are defined on the half symbol grid
local SAMPLES_PER_SYMBOL = 2

-- Preamble length, and the length of the longest frame, in symbols
local PREAMBLE_SYMBOLS = 8
local LONG_FRAME_SYMBOLS = 112

-- Frame lengths, in bits
local SHORT_FRAME_BITS = 56
local LONG_FRAME_BITS = 112

-- Downlink formats at or above this value use long frames
local LONG_FRAME_FORMAT = 16

-- Mode S generator polynomial, x^24 + ... + x^3 + 1, as its bits
local CRC_POLY_BITS = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1,
}
local CRC_BITS = 24

-- Ring buffer of two sample per symbol envelope samples, and the scratch
-- window holding one candidate preamble and frame
local BUFFER_LENGTH = 1024
local WINDOW_LENGTH = PREAMBLE_SYMBOLS * SAMPLES_PER_SYMBOL + LONG_FRAME_SYMBOLS * SAMPLES_PER_SYMBOL + 2

-- Fixed point phase correction scale, where 16384 is unity
local PHASE_SCALE = 16384

-- Two adjacent samples closer together than this fraction of the preamble
-- pulse level are treated as identical, and repeat the previous bit decision
local GLITCH_FRACTION = 0.006

-- Largest number of bit errors corrected from the syndrome of a frame. The
-- Mode S parity is a linear code, so the syndrome of a bit error is the
-- syndrome the frame would have with only that bit set, and the syndromes of
-- two bit errors are the XOR of the two. Corrected frames are re-checked
-- against the parity before they are accepted.
local MAX_CORRECTED_BITS = 2

local ADSBDetectorBlock = block.factory("ADSBDetectorBlock")

function ADSBDetectorBlock:instantiate(symbol_rate)
    self.symbol_rate = symbol_rate or 1e6

    self:add_type_signature({block.Input("in", types.Float32)}, {block.Output("out", types.Bit)})
end

-- Number of bits in a frame, from its downlink format
local function frame_bits(bits)
    local df = 0

    for i = 1, 5 do
        df = df * 2 + bits[i]
    end

    return (df >= LONG_FRAME_FORMAT) and LONG_FRAME_BITS or SHORT_FRAME_BITS
end

-- Mode S syndrome of a frame: the remainder after dividing the data portion by
-- the generator polynomial, XORed with the transmitted parity field. Zero for
-- a valid frame. DF11, DF17, and DF18 announce their address in the message
-- body, so no address overlay has to be undone.
local function syndrome(bits, num_bits, work)
    for i = 1, num_bits do
        work[i] = bits[i]
    end

    for i = 1, num_bits - CRC_BITS do
        if work[i] == 1 then
            for j = 1, CRC_BITS + 1 do
                work[i + j - 1] = bit.bxor(work[i + j - 1], CRC_POLY_BITS[j])
            end
        end
    end

    local value = 0

    for i = num_bits - CRC_BITS + 1, num_bits do
        value = value * 2 + work[i]
    end

    return value
end

local function scale_sample(value, scale)
    return value * scale / PHASE_SCALE
end

-- Correction table for a frame length, mapping the syndrome of one or two bit
-- errors to the bit positions that produced it. Built on first use, since
-- building it costs more than most invocations of the module would save.
local correct_syndromes = {}

local function correction_table(num_bits)
    local table_ = correct_syndromes[num_bits]

    if table_ then
        return table_
    end

    table_ = {}

    local work = {}
    local frame = {}
    local single = {}

    for i = 1, num_bits do
        for k = 1, num_bits do
            frame[k] = 0
        end
        frame[i] = 1
        single[i] = syndrome(frame, num_bits, work)
    end

    for i = 1, num_bits do
        table_[single[i]] = {i}
    end

    if MAX_CORRECTED_BITS > 1 then
        for i = 1, num_bits do
            for j = i + 1, num_bits do
                local value = bit.bxor(single[i], single[j])
                if table_[value] == nil then
                    table_[value] = {i, j}
                end
            end
        end
    end

    correct_syndromes[num_bits] = table_

    return table_
end

-- Flip the bits that a one or two bit error syndrome accounts for, returning
-- true if the frame was corrected
local function correct_frame(bits, num_bits, value)
    local positions = correction_table(num_bits)[value]

    if not positions then
        return false
    end

    for i = 1, #positions do
        bits[positions[i]] = bit.bxor(bits[positions[i]], 1)
    end

    return true
end

function ADSBDetectorBlock:initialize()
    self.symbol_period = self:get_rate() / self.symbol_rate

    assert(self.symbol_period >= SAMPLES_PER_SYMBOL,
           "Sample rate must be at least 2 samples per symbol")

    -- Linear interpolation resampler onto the two samples per symbol grid
    self.resample_step = self.symbol_period / SAMPLES_PER_SYMBOL
    self.resample_position = 0.0
    self.resample_previous = nil
    self.input_count = 0

    -- Ring buffer of two sample per symbol envelope samples
    self.buffer_mask = BUFFER_LENGTH - 1
    self.buffer = ffi.new("float[?]", BUFFER_LENGTH)
    self.count = 0

    -- Scratch window holding a candidate preamble and frame. Window index `w`
    -- holds the ring buffer sample at `start - 1 + w`, so window index `o + 1`
    -- holds the sample at frame offset `o`.
    self.window = ffi.new("float[?]", WINDOW_LENGTH)

    self.bits = {}
    self.work = {}

    self.out = types.Bit.vector()
end

-- Test the preamble at the start of the scratch window. Returns the pulse
-- level, or nil if the window does not hold a preamble.
function ADSBDetectorBlock:preamble()
    local window = self.window

    local m0, m1, m2, m3 = window[1], window[2], window[3], window[4]
    local m6, m7, m8, m9 = window[7], window[8], window[9], window[10]

    if not (m0 > m1 and m1 < m2 and m2 > m3 and m3 < m0 and
            window[5] < m0 and window[6] < m0 and m6 < m0 and
            m7 > m8 and m8 < m9 and m9 > m6) then
        return nil
    end

    local high = (m0 + m2 + m7 + m9) / 6

    -- The samples between the two halves of the preamble must be low
    if window[5] >= high or window[6] >= high then
        return nil
    end

    -- The space between the preamble and the first bit must be low
    if window[12] >= high or window[13] >= high or window[14] >= high or window[15] >= high then
        return nil
    end

    return high
end

-- Slice the frame bits out of the scratch window, from the first frame sample
-- at window index PREAMBLE_SYMBOLS * SAMPLES_PER_SYMBOL + 1
function ADSBDetectorBlock:slice(high)
    local window = self.window
    local bits = self.bits
    local glitch = high * GLITCH_FRACTION
    local first = PREAMBLE_SYMBOLS * SAMPLES_PER_SYMBOL

    for i = 0, LONG_FRAME_BITS - 1 do
        local low = window[first + 1 + 2 * i]
        local high_sample = window[first + 2 + 2 * i]

        if i > 0 and math.abs(low - high_sample) < glitch then
            bits[i + 1] = bits[i]
        elseif low > high_sample then
            bits[i + 1] = 1
        else
            bits[i + 1] = 0
        end
    end

    return bits
end

-- Estimate whether the frame is sampled early or late from the energy that
-- leaks into the preamble samples either side of the pulses, and rescale each
-- frame sample by the amount it is expected to be off. Original algorithm by
-- Oliver Jowett, as used by dump1090.
function ADSBDetectorBlock:phase_correct()
    local window = self.window
    local first = PREAMBLE_SYMBOLS * SAMPLES_PER_SYMBOL

    local on_time = window[1] + window[3] + window[8] + window[10]
    local early = (window[0] + window[7]) * 2
    local late = (window[4] + window[11]) * 2

    if on_time <= 0 then
        return
    end

    local up, down

    if early > late then
        up = PHASE_SCALE + PHASE_SCALE * early / (early + on_time)
        down = PHASE_SCALE - PHASE_SCALE * early / (early + on_time)

        -- Trailing samples are low, so the last one is scaled up, and the
        -- frame is then walked backwards
        local last = first + LONG_FRAME_SYMBOLS * SAMPLES_PER_SYMBOL - 1
        window[last + 1] = scale_sample(window[last + 1], up)

        for i = last - 1, first + 2, -2 do
            if window[i + 1] > window[i + 2] then
                window[i] = scale_sample(window[i], down)
            else
                window[i] = scale_sample(window[i], up)
            end
        end
    else
        up = PHASE_SCALE + PHASE_SCALE * late / (late + on_time)
        down = PHASE_SCALE - PHASE_SCALE * late / (late + on_time)

        -- Leading samples are low, so the first one is scaled up, and the
        -- frame is then walked forwards
        window[first + 1] = scale_sample(window[first + 1], up)

        for i = first, first + LONG_FRAME_SYMBOLS * SAMPLES_PER_SYMBOL - 2, 2 do
            if window[i + 1] > window[i + 2] then
                window[i + 3] = scale_sample(window[i + 3], up)
            else
                window[i + 3] = scale_sample(window[i + 3], down)
            end
        end
    end
end

-- Check a sliced frame of `num_bits` bits against the parity, correcting a
-- small number of bit errors from the syndrome. A corrected frame must still
-- agree with its own downlink format, so that the frame length the framer
-- derives matches the one that was validated.
function ADSBDetectorBlock:check(num_bits)
    local bits = self.bits
    local value = syndrome(bits, num_bits, self.work)

    if value == 0 then
        return true
    end

    if not correct_frame(bits, num_bits, value) then
        return false
    end

    return frame_bits(bits) == num_bits
end

-- Demodulate the frame whose preamble starts at ring buffer sample `start`,
-- appending its bits to `out` if it is a valid frame
function ADSBDetectorBlock:demodulate(start, out)
    local buffer, mask = self.buffer, self.buffer_mask
    local window = self.window

    for w = 0, WINDOW_LENGTH - 1 do
        window[w] = buffer[bit.band(start - 1 + w, mask)]
    end

    local high = self:preamble()

    if not high then
        return
    end

    local bits = self:slice(high)
    local num_bits = frame_bits(bits)

    if not self:check(num_bits) then
        -- Retry with the sampling phase corrected
        self:phase_correct()

        bits = self:slice(high)
        num_bits = frame_bits(bits)

        if not self:check(num_bits) then
            return
        end
    end

    for i = 1, num_bits do
        out:append(types.Bit(bits[i]))
    end
end

-- Push one two sample per symbol sample, and demodulate the candidate preamble
-- that has just become complete
function ADSBDetectorBlock:push(sample, out)
    self.buffer[bit.band(self.count, self.buffer_mask)] = sample
    self.count = self.count + 1

    local start = self.count - PREAMBLE_SYMBOLS * SAMPLES_PER_SYMBOL - LONG_FRAME_SYMBOLS * SAMPLES_PER_SYMBOL

    if start >= 1 then
        self:demodulate(start, out)
    end
end

function ADSBDetectorBlock:process(x)
    local out = self.out:resize(0)
    local step = self.resample_step

    for i = 0, x.length-1 do
        local sample = x.data[i].value

        if self.resample_previous then
            -- Emit every two sample per symbol point this input sample has
            -- now passed, interpolating linearly between input samples
            while self.resample_position <= self.input_count do
                local f = self.resample_position - (self.input_count - 1)
                self:push(self.resample_previous + (sample - self.resample_previous) * f, out)
                self.resample_position = self.resample_position + step
            end
        else
            self.resample_position = 0.0
            self:push(sample, out)
            self.resample_position = step
        end

        self.resample_previous = sample
        self.input_count = self.input_count + 1
    end

    return out
end

return ADSBDetectorBlock
