---
-- Demodulate and decode ADS-B frames from a 1090 MHz Mode S extended
-- squitter signal.
--
-- The signal envelope is computed, the PPM preamble is detected, complete
-- Mode S frames are sliced out of the envelope, and the frames are validated
-- and decoded.
--
-- The input must be sampled at two or more samples per symbol.
--
-- @category Receivers
-- @block ADSBReceiver
-- @tparam[opt={}] table options Additional options, specifying:
--                               * `symbol_rate` (number, symbol rate in
--                                 symbols per second, default 1e6)
--                               * `reference` (table, receiver position with
--                                 `lat` and `lon` keys, required for surface
--                                 positions)
--
-- @signature in:ComplexFloat32 > out:ADSBFrameType
--
-- @usage
-- local receiver = radio.ADSBReceiver({reference = {lat = 51.99, lon = 4.375}})
-- local sink = radio.PrintSink()
-- top:connect(src, receiver, sink)

local block = require('radio.core.block')
local types = require('radio.types')
local blocks = require('radio.blocks')

local ADSBReceiver = block.factory("ADSBReceiver", blocks.CompositeBlock)

function ADSBReceiver:instantiate(options)
    blocks.CompositeBlock.instantiate(self)

    options = options or {}

    local magnitude = blocks.ComplexMagnitudeBlock()
    local detector = blocks.ADSBDetectorBlock(options.symbol_rate)
    local framer = blocks.ADSBFramerBlock(options)

    self:connect(magnitude, detector, framer)

    self:add_type_signature({block.Input("in", types.ComplexFloat32)},
                            {block.Output("out", blocks.ADSBFramerBlock.ADSBFrameType)})

    self:connect(self, "in", magnitude, "in")
    self:connect(self, "out", framer, "out")
end

return ADSBReceiver
