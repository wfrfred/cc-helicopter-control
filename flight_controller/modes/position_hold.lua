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

    return common.status(true)
end

function Hold:exit() end

function Hold:terms()
    local terms = horizontalVector(self.position)

    terms.height = self.lastHeight
    terms.heading = {
        target = self.heading,
        rate = 0.0,
        active = true,
        pending = false,
        source = "position_hold",
    }
    terms.lock = {
        height = self.lastHeight.source,
        heading = "position_hold",
    }

    return terms
end

function Hold:target(input)
    local positionError = self.position - horizontalVector(input.state.world.position)
    local position = common.frdFromWorld(positionError, self.heading)

    return common.target({
        position = {
            forward = position.forward,
            right = position.right,
            down = self.lastHeight.active and input.state.body.pose.height - self.lastHeight.target or nil,
        },
        feedforward = {
            down = -self.lastHeight.rate,
        },
        attitude = {
            yaw = self.heading,
        },
    })
end

return position_hold
