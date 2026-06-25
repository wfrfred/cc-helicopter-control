local attitude_math = require("lib.attitude_math")
local feedforward = require("lib.feedforward")
local pid = require("lib.pid")

local attitude = {}

local Attitude = {}
Attitude.__index = Attitude

local function updateAngle(axisAnglePid, targetAngle, currentRate, dt)
    return axisAnglePid:update({
        target = targetAngle,
        current = 0.0,
        error = targetAngle,
        derivative = -currentRate,
        dt = dt,
    })
end

local function updateRate(axisRatePid, targetRate, currentRate, dt)
    return axisRatePid:update({
        target = targetRate,
        current = currentRate,
        dt = dt,
    })
end

function attitude.new(control)
    local controllers = {
        roll = {
            angle = pid.new(control.pid.attitude.roll.angle),
            rate = pid.new(control.pid.attitude.roll.rate),
        },
        pitch = {
            angle = pid.new(control.pid.attitude.pitch.angle),
            rate = pid.new(control.pid.attitude.pitch.rate),
        },
        yaw = {
            angle = pid.new(control.pid.attitude.yaw.angle),
            rate = pid.new(control.pid.attitude.yaw.rate),
        },
    }

    controllers.roll.rate:setFeedforward(
        feedforward.linear(
            control.attitude.rate_feedforward.roll.gain,
            control.attitude.rate_feedforward.roll.bias
        )
    )
    controllers.pitch.rate:setFeedforward(
        feedforward.linear(
            control.attitude.rate_feedforward.pitch.gain,
            control.attitude.rate_feedforward.pitch.bias
        )
    )
    controllers.yaw.rate:setFeedforward(
        feedforward.linear(control.attitude.rate_feedforward.yaw.gain)
    )

    return setmetatable({
        control = control,
        controllers = controllers,
        lastTerms = {},
    }, Attitude)
end

function Attitude:update(input)
    local state = input.state
    local target = input.target
    local externalFeedforward = input.feedforward
    local dt = input.dt
    local rates = state.body.angular.velocity
    local rollAngleFf = externalFeedforward.angle.roll
    local pitchAngleFf = externalFeedforward.angle.pitch
    local yawAngleFf = externalFeedforward.angle.yaw
    local rollRateFf = externalFeedforward.rate.roll
    local pitchRateFf = externalFeedforward.rate.pitch
    local yawRateFf = externalFeedforward.rate.yaw
    local bodyAttitudeError = attitude_math.attitudeError(
        state.body.orientation,
        target.orientation
    )
    local rollAngleResult = updateAngle(
        self.controllers.roll.angle,
        bodyAttitudeError.roll,
        rates.roll,
        dt
    )
    local pitchAngleResult = updateAngle(
        self.controllers.pitch.angle,
        bodyAttitudeError.pitch,
        rates.pitch,
        dt
    )
    local yawAngleResult = updateAngle(
        self.controllers.yaw.angle,
        bodyAttitudeError.yaw,
        rates.yaw,
        dt
    )
    local rollRateTarget = rollAngleResult.output + rollAngleFf
    local pitchRateTarget = pitchAngleResult.output + pitchAngleFf
    local yawRateTarget = yawAngleResult.output + yawAngleFf
    local rollRateResult = updateRate(
        self.controllers.roll.rate,
        rollRateTarget,
        rates.roll,
        dt
    )
    local pitchRateResult = updateRate(
        self.controllers.pitch.rate,
        pitchRateTarget,
        rates.pitch,
        dt
    )
    local yawRateResult = updateRate(
        self.controllers.yaw.rate,
        yawRateTarget,
        rates.yaw,
        dt
    )
    local rollCommand = rollRateResult.output + rollRateFf
    local pitchCommand = pitchRateResult.output + pitchRateFf
    local yawCommand = yawRateResult.output + yawRateFf

    self.lastTerms = {
        target = {
            orientation = target.orientation,
            roll = {
                angle = rollAngleResult.target,
                rate = rollRateTarget,
            },
            pitch = {
                angle = pitchAngleResult.target,
                rate = pitchRateTarget,
            },
            yaw = {
                angle = yawAngleResult.target,
                rate = yawRateTarget,
            },
        },
        current = {
            roll = {
                angle = rollAngleResult.current,
                rate = rates.roll,
            },
            pitch = {
                angle = pitchAngleResult.current,
                rate = rates.pitch,
            },
            yaw = {
                angle = yawAngleResult.current,
                rate = rates.yaw,
            },
        },
        error = {
            roll = {
                angle = rollAngleResult.error,
                rate = rollRateResult.error,
            },
            pitch = {
                angle = pitchAngleResult.error,
                rate = pitchRateResult.error,
            },
            yaw = {
                angle = yawAngleResult.error,
                rate = yawRateResult.error,
            },
        },
        terms = {
            roll = {
                angle = self.controllers.roll.angle:terms(),
                rate = self.controllers.roll.rate:terms(),
            },
            pitch = {
                angle = self.controllers.pitch.angle:terms(),
                rate = self.controllers.pitch.rate:terms(),
            },
            yaw = {
                angle = self.controllers.yaw.angle:terms(),
                rate = self.controllers.yaw.rate:terms(),
            },
        },
        feedforward = {
            angle = {
                roll = rollAngleFf,
                pitch = pitchAngleFf,
                yaw = yawAngleFf,
            },
            rate = {
                roll = rollRateFf,
                pitch = pitchRateFf,
                yaw = yawRateFf,
            },
        },
    }

    return {
        roll = rollCommand,
        pitch = pitchCommand,
        yaw = yawCommand,
    }
end

function Attitude:terms()
    return self.lastTerms
end

return attitude
