local attitude_math = require("lib.attitude_math")
local feedforward = require("lib.feedforward")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local attitude = {}

local Attitude = {}
Attitude.__index = Attitude

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
            rate = pid.new(control.pid.attitude.roll.rate),
        },
        pitch = {
            rate = pid.new(control.pid.attitude.pitch.rate),
        },
        yaw = {
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
        attitude = control.attitude,
        controllers = controllers,
        lastTerms = {},
    }, Attitude)
end

function Attitude:update(input)
    local state = input.state
    local commanded = input.commanded
    local heading = input.heading
    local headingError = input.headingError
    local dt = input.dt
    local rates = state.body.angular.velocity
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
    local targetRates = attitude_math.bodyRateCommand(
        state.body.orientation,
        attitudeTarget.orientation,
        self.attitude.time_constant
    )
    local rollRateResult = updateRate(
        self.controllers.roll.rate,
        targetRates.roll,
        rates.roll,
        dt
    )
    local pitchRateResult = updateRate(
        self.controllers.pitch.rate,
        targetRates.pitch,
        rates.pitch,
        dt
    )
    local yawRateResult = updateRate(
        self.controllers.yaw.rate,
        targetRates.yaw,
        rates.yaw,
        dt
    )

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
                rate = targetRates.roll,
            },
            pitch = {
                rate = targetRates.pitch,
            },
            yaw = {
                rate = targetRates.yaw,
            },
        },
        current = {
            roll = {
                rate = rates.roll,
            },
            pitch = {
                rate = rates.pitch,
            },
            yaw = {
                rate = rates.yaw,
            },
            heading = {
                angle = state.navigation.heading.angle,
            },
        },
        error = {
            roll = {
                angle = bodyAttitudeError.roll,
                rate = rollRateResult.error,
            },
            pitch = {
                angle = bodyAttitudeError.pitch,
                rate = pitchRateResult.error,
            },
            yaw = {
                angle = bodyAttitudeError.yaw,
                rate = yawRateResult.error,
            },
            heading = {
                angle = headingError,
            },
        },
        terms = {
            roll = {
                rate = self.controllers.roll.rate:terms(),
            },
            pitch = {
                rate = self.controllers.pitch.rate:terms(),
            },
            yaw = {
                rate = self.controllers.yaw.rate:terms(),
            },
        },
    }

    return {
        roll = rollRateResult.output,
        pitch = pitchRateResult.output,
        yaw = yawRateResult.output,
    }
end

function Attitude:terms()
    return self.lastTerms
end

return attitude
