--- this illustrates async radio control using fltk

-- install deps with `luarocks install fltk4lua`
-- run with `./luaradio examples/rtlsdr_wbfm_stereo_gui.lua`

local fl = require 'fltk4lua'
local radio = require 'radio'

local frequency = 90.7
local tune_offset = -250e3

-- Radio Blocks
local source = radio.RtlSdrSource((frequency * 1e6) + tune_offset, 1102500)
local tuner = radio.TunerBlock(tune_offset, 200e3, 5)
local fm_demod = radio.FrequencyDiscriminatorBlock(1.25)
local af_filter = radio.LowpassFilterBlock(128, 15e3)
local af_deemphasis = radio.FMDeemphasisFilterBlock(75e-6)
local af_downsampler = radio.DownsamplerBlock(5)
local sink = radio.PulseAudioSink(1)

-- Radio Connections
local top = radio.CompositeBlock()
top:connect(source, tuner, fm_demod, af_filter, af_deemphasis, af_downsampler, sink)
top:start()

-- GUI
local window = fl.Window( 260, 60, 'Mono FM Radio' )
local spinner_frequency = fl.Spinner( 140, 20, 100, 30, 'Frequency (MHz):' )
spinner_frequency:range( 2.2, 1100 )
spinner_frequency.value = frequency
window:end_group()

-- TODO: I get error about release, but it seems needed to stop radio
function window:callback()
    -- handle exit, release radio
    top:stop()
    sink:release()
end

function spinner_frequency:callback()
    print(spinner_frequency.value)
    source:setFreqency(spinner_frequency.value)
end


window:show()

fl.run()