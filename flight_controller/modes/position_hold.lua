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
    self.lastHeight = self.height:update({
        input = ctx.input.manual.velocity.up,
        value = ctx.state.body.pose.height,
        rate = ctx.state.world.velocity.y,
        dt = ctx.dt,
    })
end

function Hold:exit() end

function Hold:terms(state)
    local terms = horizontalVector(self.position)
    local height = self.lastHeight
    local headingError = state == nil and 0.0
        or mathx.wrapPi(self.heading - state.navigation.heading.angle)

    terms.height = {
        target = height.target,
        rate = height.rate,
        error = height.error,
    }
    terms.heading = {
        target = self.heading,
        rate = 0.0,
        error = headingError,
    }

    return terms
end

function Hold:target(ctx)
    local positionError = self.position - horizontalVector(ctx.state.world.position)
    local position = common.frdFromWorld(positionError, self.heading)
    local target = common.target("position")

    target.horizontal.position.forward = position.forward
    target.horizontal.position.right = position.right

    if self.lastHeight.active then
        target.altitude.position = ctx.state.body.pose.height - self.lastHeight.target
    end

    target.altitude.feedforward.position = -self.lastHeight.rate
    target.yaw.angle = self.heading

    return target
end

return position_hold
