local block = require('radio.core.block')
local types = require('radio.types')

local RealToComplexBlock = block.factory("RealToComplexBlock")

function RealToComplexBlock:instantiate()
    self:add_type_signature({block.Input("in", types.Float32)}, {block.Output("out", types.ComplexFloat32)})
end

function RealToComplexBlock:process(x)
    local out = types.ComplexFloat32.vector(x.length)

    for i = 0, x.length-1 do
        out.data[i].real = x.data[i].value
    end

    return out
end

return RealToComplexBlock
