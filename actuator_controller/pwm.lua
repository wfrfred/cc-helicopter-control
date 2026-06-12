local protocol = require("lib.protocol")

local pwm = {}

local levels = {}
local accumulators = {}
local outputs = {}

local function normalize(value)
    return protocol.clamp(tonumber(value) or 0, 0, 15)
end

function pwm.set(side, value)
    levels[side] = normalize(value)
    accumulators[side] = accumulators[side] or 0
end

function pwm.get(side)
    return levels[side] or 0
end

function pwm.snapshot()
    local out = {}

    for side, level in pairs(levels) do
        out[side] = {
            level = level,
            output = outputs[side] or 0,
        }
    end

    return out
end

function pwm.run()
    while true do
        for side, level in pairs(levels) do
            local base = math.floor(level)
            local frac = level - base
            local acc = (accumulators[side] or 0) + frac
            local value = base

            if acc >= 1.0 and base < 15 then
                acc = acc - 1.0
                value = base + 1
            end

            accumulators[side] = acc
            outputs[side] = value
            redstone.setAnalogOutput(side, value)
        end

        sleep(0)
    end
end

return pwm
