local common = require("modes.common")
local attitude_math = require("lib.attitude_math")

local manual = {}

function manual.target(input)
    local target = common.base(input)

    target.attitude.roll = input.mode.manualAttitude.roll
    target.attitude.pitch = input.mode.manualAttitude.pitch

    if input.heading.source == "manual" then
        target.attitude.feedforward.angle = attitude_math.bodyRatesFromEulerRates(
            input.state.body.pose.roll,
            input.state.body.pose.pitch,
            {
                heading = input.heading.rate,
            }
        )
    end

    return target
end

return manual
