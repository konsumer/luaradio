local radio = require('radio')

local application = {
    name = "rx_adsb",
    description = "ADS-B Receiver (1090 MHz Mode S Extended Squitter)",
    supported_inputs = {
        {"rtlsdr", defaults = {_rate = 2400000}},
        {"airspy", defaults = {_rate = 6000000}},
        {"bladerf", defaults = {_rate = 2400000}},
        {"hackrf", defaults = {_rate = 2400000}},
        {"hydrasdr", defaults = {_rate = 2400000}},
        {"sdrplay", defaults = {_rate = 2400000}},
        {"uhd", defaults = {_rate = 2400000}},
        {"soapysdr"},
        {"networkclient"},
        {"networkserver"},
        {"iqfile"},
    },
    supported_outputs = {
        {"text", defaults = {timestamp = true}},
        {"json"},
    },
    arguments = {},
    options = {
        {"frequency", "f", true, "Center frequency in Hz (default 1090e6)"},
        {"sample-rate", "r", true, "Sample rate in Hz (default depends on input)"},
        {"reference", nil, true, "Receiver reference position, LAT,LON (enables surface positions)"},
        {"color", nil, false, "Give each aircraft its own background colour"},
        {"no-color", nil, false, "Never use colour"},
    },
}

-- Colour is on when the text output is a terminal, and off when it is
-- redirected, so piped or logged output stays free of escape sequences. The
-- conventional NO_COLOR environment variable disables it as well.
local function want_color(args)
    if args["no-color"] then
        return false
    elseif args.color then
        return true
    elseif os.getenv("NO_COLOR") ~= nil then
        return false
    end

    local ffi = require('ffi')
    pcall(ffi.cdef, "int isatty(int fd);")

    local ok, isatty = pcall(function() return ffi.C.isatty(1) end)

    return ok and isatty ~= 0
end

function application.run(input, output, args)
    local frequency = tonumber(args.frequency) or 1090e6
    local rate = tonumber(args['sample-rate']) or input.options._rate

    local options = {}

    if args.reference then
        local reference_lat, reference_lon = string.match(args.reference, "^%s*(-?[%d%.]+)%s*,%s*(-?[%d%.]+)%s*$")
        if not reference_lat then
            error("Invalid reference position \"" .. args.reference .. "\", expected LAT,LON")
        end

        options.reference = {lat = tonumber(reference_lat), lon = tonumber(reference_lon)}
    end

    radio.ADSBFramerBlock.ADSBFrameType.color = want_color(args)

    local source = input.block(frequency, rate)
    local receiver = radio.ADSBReceiver(options)
    local sink = output.block()

    radio.debug.printf("[rx_adsb] Source sample rate %u Hz\n", source:get_rate())

    local top = radio.CompositeBlock()
    top:connect(source, receiver, sink)
    top:run()
end

return application
