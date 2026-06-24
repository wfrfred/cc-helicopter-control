local common = require("modes.common")
local attitude_math = require("lib.attitude_math")
local lock = require("modes.lock")
local mathx = require("lib.mathx")

local manual = {}

local Manual = {}
Manual.__index = Manual

local function moveToward(x, target, rate, dt)
    local d = target - x
    local step = rate * dt

    if math.abs(d) <= step then
        return target
    end

    if d > 0 then
        return x + step
    end

    return x - step
end

function manual.active(input)
    return input.manual.attitude.roll ~= 0.0
        or input.manual.attitude.pitch ~= 0.0
        or input.manual.heading.rate ~= 0.0
end

function manual.new(initialState, control)
    local self = setmetatable({
        control = control,
        height = lock.new({
            initial = initialState.body.pose.height,
            target_rate = control.vertical.target_rate,
            rate_deadband = control.vertical.lock.speed_deadband,
            relock_timeout = control.vertical.lock.relock_timeout,
        }),
        heading = lock.new({
            initial = initialState.navigation.heading.angle,
            target_rate = control.heading.target_rate,
            rate_deadband = control.heading.lock.rate_deadband,
            relock_timeout = control.heading.lock.relock_timeout,
            normalize = mathx.wrapPi,
            error = function(target, value)
                return mathx.wrapPi(target - value)
            end,
        }),
        lastHeight = nil,
        lastHeading = nil,
        roll = control.attitude.home.roll,
        pitch = control.attitude.home.pitch,
    }, Manual)

    self.lastHeight = self.height:locked(initialState.body.pose.height)
    self.lastHeading = self.heading:locked(initialState.navigation.heading.angle)

    return self
end

function Manual:enter(ctx)
    local control = self.control
    local pose = ctx.state.body.pose
    local roll = pose.roll or control.attitude.home.roll
    local pitch = pose.pitch or control.attitude.home.pitch

    self.roll = mathx.clamp(
        roll,
        -control.attitude.limit.roll,
        control.attitude.limit.roll
    )
    self.pitch = mathx.clamp(
        pitch,
        -control.attitude.limit.pitch,
        control.attitude.limit.pitch
    )
    if ctx.input.manual.velocity.up == 0.0 then
        self.lastHeight = self.height:locked(ctx.state.body.pose.height)
    end

    if ctx.input.manual.heading.rate == 0.0 then
        self.lastHeading = self.heading:locked(ctx.state.navigation.heading.angle)
    end
end

function Manual:exit() end

function Manual:update(ctx)
    local input = ctx.input

    self.lastHeight = self.height:update({
        input = input.manual.velocity.up,
        value = ctx.state.body.pose.height,
        rate = ctx.state.world.velocity.y,
        dt = ctx.dt,
    })
    self.lastHeading = self.heading:update({
        input = input.manual.heading.rate,
        value = ctx.state.navigation.heading.angle,
        rate = ctx.state.navigation.heading.rate,
        dt = ctx.dt,
    })

    local dt = ctx.dt
    local control = self.control
    local roll = input.manual.attitude.roll
    local pitch = input.manual.attitude.pitch

    if roll ~= 0.0 then
        self.roll = mathx.clamp(
            self.roll + roll * control.attitude.target_rate.roll * dt,
            -control.attitude.limit.roll,
            control.attitude.limit.roll
        )
    else
        self.roll = moveToward(
            self.roll,
            control.attitude.home.roll,
            control.attitude.center_rate.roll,
            dt
        )
    end

    if pitch ~= 0.0 then
        self.pitch = mathx.clamp(
            self.pitch + pitch * control.attitude.target_rate.pitch * dt,
            -control.attitude.limit.pitch,
            control.attitude.limit.pitch
        )
    else
        self.pitch = moveToward(
            self.pitch,
            control.attitude.home.pitch,
            control.attitude.center_rate.pitch,
            dt
        )
    end

    return common.status(manual.active(input))
end

function Manual:terms()
    return {
        roll = self.roll,
        pitch = self.pitch,
        height = self.lastHeight,
        heading = self.lastHeading,
        lock = {
            height = self.lastHeight.source,
            heading = self.lastHeading.source,
        },
    }
end

function Manual:target(input)
    local height = self.lastHeight
    local heading = self.lastHeading
    local target = common.target({
        position = {
            down = height.active and input.state.body.pose.height - height.target or nil,
        },
        feedforward = {
            down = -height.rate,
        },
        attitude = {
            roll = self.roll,
            pitch = self.pitch,
            yaw = heading.active and heading.target or nil,
        },
    })

    if heading.source == "manual" then
        target.attitude.feedforward.angle = attitude_math.bodyRatesFromEulerRates(
            input.state.body.pose.roll,
            input.state.body.pose.pitch,
            {
                heading = heading.rate,
            }
        )
    end

    return target
end

return manual
