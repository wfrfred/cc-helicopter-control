local common = require("modes.common")

local position_hold = {}

function position_hold.target(input)
    local target = common.base(input)

    target.world.position = input.mode.positionTarget

    return target
end

return position_hold
