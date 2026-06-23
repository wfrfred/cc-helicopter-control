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

function Hold:enter(ctx)
    self.position = horizontalVector(ctx.state.world.position)
end

function Hold:update(ctx)
    if ctx.current ~= "position_hold" then
        return {
            active = false,
        }
    end

    return {
        active = true,
    }
end

function Hold:exit() end

function Hold:snapshot()
    return horizontalVector(self.position)
end

function Hold:terms()
    return self:snapshot()
end

function Hold:target(input)
    local target = common.base(input)

    target.world.position = self:snapshot()

    return target
end

return position_hold
