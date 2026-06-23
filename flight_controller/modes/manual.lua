local common = require("modes.common")
local attitude_math = require("lib.attitude_math")
local axis_locks = require("modes.axis_locks")
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
    return setmetatable({
        control = control,
        locks = axis_locks.new(initialState, control),
        roll = control.attitude.home.roll,
        pitch = control.attitude.home.pitch,
    }, Manual)
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
    self.locks:enter(ctx)
end

function Manual:exit() end

function Manual:update(ctx)
    local input = ctx.input

    if ctx.current ~= "manual" then
        return {
            active = manual.active(input),
        }
    end

    self.locks:update(ctx)

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
        active = manual.active(input),
    }
end

function Manual:snapshot()
    return {
        roll = self.roll,
        pitch = self.pitch,
    }
end

function Manual:terms()
    return self:snapshot()
end

function Manual:target(input)
    local heading = self.locks:headingTarget()
    local target = common.base({
        source = input.source,
        vertical = self.locks:verticalTarget(),
        heading = heading,
    })

    target.attitude.roll = self.roll
    target.attitude.pitch = self.pitch

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

function Manual:axisTerms()
    return self.locks:terms()
end

return manual
