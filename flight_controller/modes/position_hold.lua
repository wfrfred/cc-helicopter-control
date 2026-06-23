local common = require("modes.common")
local lock = require("modes.lock")
local mathx = require("lib.mathx")

local position_hold = {}

local Hold = {}
Hold.__index = Hold

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

function position_hold.new(initialState, control)
    local self = setmetatable({
        height = lock.new({
            initial = initialState.body.pose.height,
            target_rate = control.vertical.target_rate,
            rate_deadband = control.vertical.lock.speed_deadband,
            relock_timeout = control.vertical.lock.relock_timeout,
        }),
        lastHeight = nil,
        heading = mathx.wrapPi(initialState.navigation.heading.angle),
        position = horizontalVector(initialState.world.position),
    }, Hold)

    self.lastHeight = self.height:locked(initialState.body.pose.height)

    return self
end

function Hold:enter(ctx)
    self.position = horizontalVector(ctx.state.world.position)
    self.heading = mathx.wrapPi(ctx.state.navigation.heading.angle)

    if ctx.input.manual.velocity.up == 0.0 then
        self.lastHeight = self.height:locked(ctx.state.body.pose.height)
    end
end

function Hold:update(ctx)
    if ctx.current ~= "position_hold" then
        return {
            active = false,
        }
    end

    self.lastHeight = self.height:update({
        input = ctx.input.manual.velocity.up,
        value = ctx.state.body.pose.height,
        rate = ctx.state.world.velocity.y,
        dt = ctx.dt,
    })

    return {
        active = true,
    }
end

function Hold:exit() end

function Hold:snapshot()
    return horizontalVector(self.position)
end

local function verticalTarget(result)
    return {
        height = result.target,
        speed = result.rate,
        active = result.active,
        pending = result.pending,
        error = result.error,
        source = result.source,
    }
end

local function headingTarget(self, state)
    return {
        angle = self.heading,
        rate = 0.0,
        active = true,
        pending = false,
        error = mathx.wrapPi(self.heading - state.navigation.heading.angle),
        source = "position_hold",
    }
end

local function targetControl(self, state)
    local heading = headingTarget(self, state)

    return {
        height = verticalTarget(self.lastHeight),
        heading = heading,
        lock = {
            height = self.lastHeight.source,
            heading = heading.source,
        },
    }
end

function Hold:terms(state)
    local terms = self:snapshot()

    if state ~= nil then
        terms.control = targetControl(self, state)
    end

    return terms
end

function Hold:target(input)
    local terms = targetControl(self, input.state)
    local target = common.base({
        source = input.source,
        vertical = terms.height,
        heading = terms.heading,
    })

    target.world.position = self:snapshot()

    return target
end

return position_hold
