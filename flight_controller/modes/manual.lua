local common = require("modes.common")
local attitude_math = require("lib.attitude_math")
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

function manual.new(control)
    return setmetatable({
        control = control,
        roll = control.attitude.home.roll,
        pitch = control.attitude.home.pitch,
    }, Manual)
end

function Manual:enter() end

function Manual:exit() end

function Manual:update(ctx)
    local input = ctx.input

    if ctx.current ~= "manual" then
        return {
            active = manual.active(input),
        }
    end

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
    local target = common.base(input)

    target.attitude.roll = self.roll
    target.attitude.pitch = self.pitch

    if input.heading.source == "manual" then
        target.attitude.feedforward.angle = attitude_math.bodyRatesFromEulerRates(
            input.state.body.pose.roll,
            input.state.body.pose.pitch,
            {
                heading = input.heading.rate,
            }
        )
    end

    return target
end

return manual
