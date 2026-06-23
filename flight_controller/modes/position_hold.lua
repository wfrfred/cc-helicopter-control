local common = require("modes.common")
local axis_locks = require("modes.axis_locks")

local position_hold = {}

local Hold = {}
Hold.__index = Hold

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

function position_hold.new(initialState, control)
    return setmetatable({
        locks = axis_locks.new(initialState, control),
        position = horizontalVector(initialState.world.position),
    }, Hold)
end

function Hold:enter(ctx)
    self.position = horizontalVector(ctx.state.world.position)
    self.locks:enter(ctx)
end

function Hold:update(ctx)
    if ctx.current ~= "position_hold" then
        return {
            active = false,
        }
    end

    self.locks:update(ctx)

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
    local target = common.base({
        source = input.source,
        vertical = self.locks:verticalTarget(),
        heading = self.locks:headingTarget(),
    })

    target.world.position = self:snapshot()

    return target
end

function Hold:axisTerms()
    return self.locks:terms()
end

return position_hold
