local controller = require("control.controller")
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

local function heading(state)
    local forward = state.frames.navigation:basis().forward

    return mathx.wrapPi(mathx.atan2(forward.x, -forward.z))
end

local function bodyAttitude(state)
    local basis = state.frames.body:basis()
    local forwardHorizontal = vector.new(basis.forward.x, 0.0, basis.forward.z)
    local horizontal = forwardHorizontal:length()

    return {
        roll = mathx.wrapPi(mathx.atan2(-basis.right.y, -basis.down.y)),
        pitch = mathx.wrapPi(mathx.atan2(basis.forward.y, horizontal)),
    }
end

function manual.active(input)
    return input.manual.attitude.roll ~= 0.0
        or input.manual.attitude.pitch ~= 0.0
        or input.manual.heading.rate ~= 0.0
end

local function buildTerms(self)
    local height = self.height
    local heading = self.heading

    return {
        roll = self.roll,
        pitch = self.pitch,
        height = {
            target = -height.target,
            rate = -height.rate,
            error = -height.error,
        },
        heading = {
            target = heading.target,
            rate = heading.rate,
            error = heading.error,
        },
    }
end

local function buildTarget(self, ctx)
    local height = self.height
    local headingState = self.heading
    local target = controller.target("attitude")

    if height.locked then
        target.vertical.position = height.target - ctx.state.navigation.position.z
    end

    target.vertical.feedforward.position = height.rate
    target.horizontal.angle.roll = self.roll
    target.horizontal.angle.pitch = self.pitch
    target.yaw.angle = headingState.locked and headingState.target or heading(ctx.state)

    if headingState.manual then
        local angleFeedforward = ctx.state.frames.body:componentsOf(
            vector.new(0.0, -headingState.rate, 0.0)
        )

        target.horizontal.feedforward.angle.roll = angleFeedforward.x
        target.horizontal.feedforward.angle.pitch = angleFeedforward.y
        target.yaw.feedforward.angle = angleFeedforward.z
    end

    return target
end

function manual.new(initialState, control)
    local self = setmetatable({
        control = control,
        heightLock = lock.new({
            initial = initialState.navigation.position.z,
            target_rate = control.vertical.target_rate,
            rate_deadband = control.vertical.lock.speed_deadband,
            relock_timeout = control.vertical.lock.relock_timeout,
        }),
        headingLock = lock.new({
            initial = heading(initialState),
            target_rate = control.heading.target_rate,
            rate_deadband = control.heading.lock.rate_deadband,
            relock_timeout = control.heading.lock.relock_timeout,
            normalize = mathx.wrapPi,
            error = function(target, value)
                return mathx.wrapPi(target - value)
            end,
        }),
        height = nil,
        heading = nil,
        roll = control.attitude.home.roll,
        pitch = control.attitude.home.pitch,
    }, Manual)

    self.height = self.heightLock:locked(initialState.navigation.position.z)
    self.heading = self.headingLock:locked(heading(initialState))

    return self
end

function Manual:enter(ctx)
    local control = self.control
    local attitude = bodyAttitude(ctx.state)
    local roll = attitude.roll or control.attitude.home.roll
    local pitch = attitude.pitch or control.attitude.home.pitch

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
        self.height = self.heightLock:locked(ctx.state.navigation.position.z)
    end

    if ctx.input.manual.heading.rate == 0.0 then
        self.heading = self.headingLock:locked(heading(ctx.state))
    end
end

function Manual:update(ctx)
    local input = ctx.input

    self.height = self.heightLock:update({
        input = -input.manual.velocity.up,
        value = ctx.state.navigation.position.z,
        rate = ctx.state.navigation.velocity.z,
        dt = ctx.dt,
    })
    self.heading = self.headingLock:update({
        input = input.manual.heading.rate,
        value = heading(ctx.state),
        rate = ctx.state.navigation.angularVelocity.z,
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

    return {
        target = buildTarget(self, ctx),
        terms = buildTerms(self),
    }
end

function Manual:exit() end

return manual
