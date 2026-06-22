local common = require("modes.common")

local cruise = {}

function cruise.target(input)
    local target = common.base(input)

    target.world.velocity = input.mode.cruiseVelocity

    return target
end

return cruise
