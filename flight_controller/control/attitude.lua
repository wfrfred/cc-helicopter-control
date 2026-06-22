local attitude_math = require("lib.attitude_math")
local feedforward = require("lib.feedforward")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local attitude = {}

local Attitude = {}
Attitude.__index = Attitude

local function feedforwardValue(feedforward, layer, axis)
    if type(feedforward) ~= "table" or type(feedforward[layer]) ~= "table" then
        return 0.0
    end

    return feedforward[layer][axis] or 0.0
end

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

local function targetOrientation(control, currentFrame, commanded, heading)
    local fullFrame = attitude_math.frameFromPose(commanded.roll, commanded.pitch, heading)
    local full = attitude_math.quaternionFromFrame(fullFrame):normalize()
    local reducedFrame = attitude_math.reducedFrameFromTargetDown(currentFrame, fullFrame)
    local reduced = attitude_math.quaternionFromFrame(reducedFrame):normalize()
    local yawPriority = mathx.clamp(control.heading.yaw_priority, 0.0, 1.0)
    local mixed = reduced:slerp(full, yawPriority):normalize()

    return {
        roll = commanded.roll,
        pitch = commanded.pitch,
        orientation = mixed,
        fullOrientation = full,
        reducedOrientation = reduced,
        yawPriority = yawPriority,
    }
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
    local commanded = input.commanded
    local externalFeedforward = input.feedforward or {}
    local heading = input.heading
    local headingError = input.headingError
    local dt = input.dt
    local rates = state.body.angular.velocity
    local rollAngleFf = feedforwardValue(externalFeedforward, "angle", "roll")
    local pitchAngleFf = feedforwardValue(externalFeedforward, "angle", "pitch")
    local yawAngleFf = feedforwardValue(externalFeedforward, "angle", "yaw")
    local rollRateFf = feedforwardValue(externalFeedforward, "rate", "roll")
    local pitchRateFf = feedforwardValue(externalFeedforward, "rate", "pitch")
    local yawRateFf = feedforwardValue(externalFeedforward, "rate", "yaw")
    local attitudeTarget = targetOrientation(
        self.control,
        state.body.frame,
        commanded,
        heading
    )
    local bodyAttitudeError = attitude_math.attitudeError(
        state.body.orientation,
        attitudeTarget.orientation
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
        commanded = {
            roll = commanded.roll,
            pitch = commanded.pitch,
            heading = heading,
            source = commanded.source,
        },
        target = {
            orientation = attitudeTarget.orientation,
            fullOrientation = attitudeTarget.fullOrientation,
            reducedOrientation = attitudeTarget.reducedOrientation,
            yawPriority = attitudeTarget.yawPriority,
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
            heading = {
                angle = state.navigation.heading.angle,
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
            heading = {
                angle = headingError,
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
