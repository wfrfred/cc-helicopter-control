local common = require("modes.common")
local frames = require("lib.frames")
local lock = require("modes.lock")
local mathx = require("lib.mathx")

local position_hold = {}

local Hold = {}
Hold.__index = Hold

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

local function buildTerms(self, state)
    local terms = horizontalVector(self.position)
    local height = self.height
    local headingError = mathx.wrapPi(self.heading - state.navigation.heading.angle)

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

local function buildTarget(self, ctx)
    local positionError = self.position - horizontalVector(ctx.state.world.position)
    local position = frames.frdFromVector(frames.level(self.heading):componentsOf(positionError))
    local target = common.target("position")

    target.horizontal.position.forward = position.forward
    target.horizontal.position.right = position.right

    if self.height.active then
        target.altitude.position = ctx.state.body.pose.height - self.height.target
    end

    target.altitude.feedforward.position = -self.height.rate
    target.yaw.angle = self.heading

    return target
end

function position_hold.new(initialState, control)
    local self = setmetatable({
        heightLock = lock.new({
            initial = initialState.body.pose.height,
            target_rate = control.vertical.target_rate,
            rate_deadband = control.vertical.lock.speed_deadband,
            relock_timeout = control.vertical.lock.relock_timeout,
        }),
        height = nil,
        heading = mathx.wrapPi(initialState.navigation.heading.angle),
        position = horizontalVector(initialState.world.position),
    }, Hold)

    self.height = self.heightLock:locked(initialState.body.pose.height)

    return self
end

function Hold:enter(ctx)
    self.position = horizontalVector(ctx.state.world.position)
    self.heading = mathx.wrapPi(ctx.state.navigation.heading.angle)

    if ctx.input.manual.velocity.up == 0.0 then
        self.height = self.heightLock:locked(ctx.state.body.pose.height)
    end
end

function Hold:update(ctx)
    self.height = self.heightLock:update({
        input = ctx.input.manual.velocity.up,
        value = ctx.state.body.pose.height,
        rate = ctx.state.world.velocity.y,
        dt = ctx.dt,
    })

    return {
        target = buildTarget(self, ctx),
        terms = buildTerms(self, ctx.state),
    }
end

function Hold:exit() end

return position_hold
