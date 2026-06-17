local feedforward = require("lib.feedforward")
local attitude_math = require("lib.attitude_math")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

local function attitudeVerticalFactor(roll, pitch, minFactor)
    local factor = math.cos(roll) * math.cos(pitch)

    return mathx.clamp(factor, minFactor, 1.0)
end

local function bodyRatesForHeadingRate(roll, pitch, headingRate)
    local sinRoll = math.sin(roll)
    local cosRoll = math.cos(roll)
    local sinPitch = math.sin(pitch)
    local cosPitch = math.cos(pitch)

    return {
        roll = -sinPitch * headingRate,
        pitch = sinRoll * cosPitch * headingRate,
        yaw = cosRoll * cosPitch * headingRate,
    }
end

local function updateAngleRate(axis, bodyError, currentRate, targetRateFeedforward, dt)
    targetRateFeedforward = targetRateFeedforward or 0.0

    local angle = axis.angle:update({
        target = bodyError,
        current = 0.0,
        error = bodyError,
        dt = dt,
        derivative = targetRateFeedforward - currentRate,
    })
    local rate = axis.rate:update({
        target = angle.output + targetRateFeedforward,
        current = currentRate,
        dt = dt,
    })

    return {
        angle = angle,
        rate = rate,
        correctionRate = angle.output,
        feedforwardRate = targetRateFeedforward,
    }
end

function controller.new(control)
    local controllers = {
        vertical = {
            height = pid.new(control.pid.vertical.height),
            speed = pid.new(control.pid.vertical.speed),
        },
        attitude = {
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
        },
    }

    controllers.vertical.speed:setFeedforward(
        feedforward.linear(control.vertical.feedforward.gain, control.vertical.feedforward.bias)
    )
    controllers.attitude.roll.rate:setFeedforward(
        feedforward.linear(
            control.attitude.rate_feedforward.roll.gain,
            control.attitude.rate_feedforward.roll.bias
        )
    )
    controllers.attitude.pitch.rate:setFeedforward(
        feedforward.linear(
            control.attitude.rate_feedforward.pitch.gain,
            control.attitude.rate_feedforward.pitch.bias
        )
    )
    controllers.attitude.yaw.rate:setFeedforward(
        feedforward.linear(control.attitude.rate_feedforward.yaw.gain)
    )

    return setmetatable({
        collective = control.collective,
        vertical = control.vertical,
        attitude = control.attitude,
        controllers = controllers,
    }, Controller)
end

function Controller:update(input)
    local target = input.target
    local state = input.state
    local currentOrientation = state.orientation
    local pose = state.pose
    local rates = state.rates
    local vertical = state.vertical
    local attitudeTarget = target.attitude
    local verticalTarget = target.vertical
    local headingTarget = target.heading
    local height = vertical.height
    local verticalSpeed = vertical.speed
    local rollRate = rates.roll
    local pitchRate = rates.pitch
    local yawRate = rates.yaw
    local dt = input.dt
    local pids = self.controllers

    assert(type(currentOrientation) == "table", "controller state.orientation must be set")
    assert(type(attitudeTarget.roll) == "number", "controller target.attitude.roll must be number")
    assert(type(attitudeTarget.pitch) == "number", "controller target.attitude.pitch must be number")
    assert(type(attitudeTarget.orientation) == "table", "controller target.attitude.orientation must be set")

    local targetVerticalSpeed = verticalTarget.speed
    local heightErr = verticalTarget.error
    local heightResult = nil

    if verticalTarget.active then
        heightResult = pids.vertical.height:update({
            target = verticalTarget.height,
            current = height,
            dt = dt,
            derivative = -verticalSpeed,
        })
        targetVerticalSpeed = heightResult.output
        heightErr = heightResult.error
    else
        pids.vertical.height:reset()
    end

    local verticalSpeedResult = pids.vertical.speed:update({
        target = targetVerticalSpeed,
        current = verticalSpeed,
        dt = dt,
    })
    local collectiveOut = verticalSpeedResult.output
    local tiltVerticalFactor = attitudeVerticalFactor(
        pose.roll,
        pose.pitch,
        self.collective.tilt_compensation.min_factor
    )
    local tiltCompensation = 1.0 / tiltVerticalFactor
    local tiltCompensatedCollectiveOut = collectiveOut * tiltCompensation

    local headingErr = headingTarget.error
    local headingActive = headingTarget.active
    local headingRateFeedforward = bodyRatesForHeadingRate(
        attitudeTarget.roll,
        attitudeTarget.pitch,
        headingTarget.rate or 0.0
    )

    local bodyAttitudeError = attitude_math.attitudeError(
        currentOrientation,
        attitudeTarget.orientation
    )

    local rollResult = updateAngleRate(
        pids.attitude.roll,
        bodyAttitudeError.roll,
        rollRate,
        headingRateFeedforward.roll,
        dt
    )
    local pitchResult = updateAngleRate(
        pids.attitude.pitch,
        bodyAttitudeError.pitch,
        pitchRate,
        headingRateFeedforward.pitch,
        dt
    )
    local yawAngleResult = nil
    local yawCorrectionRate = 0.0

    if headingActive then
        yawAngleResult = pids.attitude.yaw.angle:update({
            target = bodyAttitudeError.yaw,
            current = 0.0,
            error = bodyAttitudeError.yaw,
            dt = dt,
            derivative = headingRateFeedforward.yaw - yawRate,
        })
        yawCorrectionRate = yawAngleResult.output
    else
        pids.attitude.yaw.angle:reset()
    end

    local targetBodyRates = {
        roll = rollResult.rate.target,
        pitch = pitchResult.rate.target,
        yaw = yawCorrectionRate + headingRateFeedforward.yaw,
    }
    local yawRateResult = pids.attitude.yaw.rate:update({
        target = targetBodyRates.yaw,
        current = yawRate,
        dt = dt,
    })

    local collective = mathx.clamp(
        tiltCompensatedCollectiveOut,
        self.collective.min,
        self.collective.max
    )

    local commands = {
        collective = collective,
        roll = rollResult.rate.output,
        pitch = pitchResult.rate.output,
        yaw = yawRateResult.output,
    }

    return {
        commands = commands,

        output = {
            commands = commands,
            collective = {
                command = commands.collective,
                feedforward = verticalSpeedResult.terms.ff,
                feedback = verticalSpeedResult.terms.raw,
                uncompensated = collectiveOut,
                tilt = {
                    compensation = tiltCompensation,
                    verticalFactor = tiltVerticalFactor,
                },
            },
            attitude = {
                roll = {
                    command = commands.roll,
                    feedforward = rollResult.rate.terms.ff,
                    feedback = rollResult.rate.terms.raw,
                    targetRate = rollResult.rate.target,
                    angleRate = rollResult.correctionRate,
                    headingRateFeedforward = rollResult.feedforwardRate,
                },
                pitch = {
                    command = commands.pitch,
                    feedforward = pitchResult.rate.terms.ff,
                    feedback = pitchResult.rate.terms.raw,
                    targetRate = pitchResult.rate.target,
                    angleRate = pitchResult.correctionRate,
                    headingRateFeedforward = pitchResult.feedforwardRate,
                },
                yaw = {
                    command = commands.yaw,
                    feedforward = yawRateResult.terms.ff,
                    feedback = yawRateResult.terms.raw,
                    targetRate = yawRateResult.target,
                    angleRate = yawCorrectionRate,
                    headingRateFeedforward = headingRateFeedforward.yaw,
                },
                kinematics = {
                    headingRateFeedforward = headingRateFeedforward,
                    targetBodyRates = targetBodyRates,
                },
            },
        },

        target = {
            vertical = {
                height = verticalTarget.height,
                speed = targetVerticalSpeed,
            },
            attitude = {
                roll = {
                    angle = rollResult.angle.target,
                    rate = rollResult.rate.target,
                },
                pitch = {
                    angle = pitchResult.angle.target,
                    rate = pitchResult.rate.target,
                },
                yaw = {
                    angle = yawAngleResult and yawAngleResult.target or 0.0,
                    rate = yawRateResult.target,
                },
            },
            heading = {
                angle = headingTarget.angle,
                rate = headingTarget.rate,
                active = headingTarget.active,
                pending = headingTarget.pending,
                source = headingTarget.source,
            },
        },

        current = {
            vertical = {
                height = height,
                speed = verticalSpeed,
            },
            attitude = {
                roll = {
                    angle = rollResult.angle.current,
                    rate = rollRate,
                },
                pitch = {
                    angle = pitchResult.angle.current,
                    rate = pitchRate,
                },
                yaw = {
                    angle = yawAngleResult and yawAngleResult.current or 0.0,
                    rate = yawRate,
                },
            },
            heading = {
                angle = pose.heading,
            },
        },

        error = {
            vertical = {
                height = heightErr,
                speed = verticalSpeedResult.error,
            },
            attitude = {
                roll = {
                    angle = rollResult.angle.error,
                    rate = rollResult.rate.error,
                },
                pitch = {
                    angle = pitchResult.angle.error,
                    rate = pitchResult.rate.error,
                },
                yaw = {
                    angle = bodyAttitudeError.yaw,
                    rate = yawRateResult.error,
                },
            },
            heading = {
                angle = headingErr,
            },
        },

        terms = {
            vertical = {
                height = {
                    result = heightResult,
                    target = verticalTarget.height,
                    current = height,
                    error = heightErr,
                    output = targetVerticalSpeed,
                    lockActive = verticalTarget.active,
                    lockPending = verticalTarget.pending,
                },
                speed = {
                    result = verticalSpeedResult,
                    tiltCompensation = tiltCompensation,
                    tiltVerticalFactor = tiltVerticalFactor,
                    uncompensated = collectiveOut,
                    output = tiltCompensatedCollectiveOut,
                },
            },

            attitude = {
                roll = rollResult,
                pitch = pitchResult,
                yaw = {
                    angle = {
                        result = yawAngleResult,
                        target = yawAngleResult and yawAngleResult.target or 0.0,
                        current = yawAngleResult and yawAngleResult.current or 0.0,
                        error = bodyAttitudeError.yaw,
                        headingError = headingErr,
                        output = yawCorrectionRate,
                        headingRateFeedforward = headingRateFeedforward.yaw,
                        targetBodyRate = targetBodyRates.yaw,
                        active = headingActive,
                        pending = headingTarget.pending,
                        headingTarget = headingTarget.angle,
                        headingCurrent = pose.heading,
                    },
                    rate = yawRateResult,
                },
            },
        },
    }
end

function Controller:pidControllers()
    local pids = self.controllers

    return {
        vertical = {
            height = pids.vertical.height,
            speed = pids.vertical.speed,
        },
        attitude = pids.attitude,
    }
end

return controller
