local common = require("modes.common")

local manual = {}

function manual.target(input)
    local target = common.base(input)

    target.attitude.roll = input.mode.manualAttitude.roll
    target.attitude.pitch = input.mode.manualAttitude.pitch

    return target
end

return manual
