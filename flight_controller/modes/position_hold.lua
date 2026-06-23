local common = require("modes.common")

local position_hold = {}

local Hold = {}
Hold.__index = Hold

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

function position_hold.new(initialState)
    return setmetatable({
        position = horizontalVector(initialState.world.position),
    }, Hold)
end

function Hold:capture(state)
    self.position = horizontalVector(state.world.position)
end

function Hold:snapshot()
    return horizontalVector(self.position)
end

function Hold:target(input)
    local target = common.base(input)

    target.world.position = self:snapshot()

    return target
end

return position_hold
