local feedforward = require("lib.feedforward")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

local function attitudeVerticalFactor(roll, pitch, minFactor)
    local factor = math.cos(roll) * math.cos(pitch)

    return mathx.clamp(factor, minFactor, 1.0)
end

local function bodyFrameFromPose(roll, pitch, heading)
    local sinHeading = math.sin(heading)
    local cosHeading = math.cos(heading)
    local sinPitch = math.sin(pitch)
    local cosPitch = math.cos(pitch)
    local sinRoll = math.sin(roll)
    local cosRoll = math.cos(roll)

    local forwardHorizontal = vector.new(sinHeading, 0.0, -cosHeading)
    local rightLevel = vector.new(cosHeading, 0.0, sinHeading)
    local worldDown = vector.new(0.0, -1.0, 0.0)
    local forward = forwardHorizontal * cosPitch + worldDown * -sinPitch
    local downLevel = forward:cross(rightLevel)

    return {
        forward = forward,
        right = rightLevel * cosRoll + downLevel * sinRoll,
        down = rightLevel * -sinRoll + downLevel * cosRoll,
    }
end

-- Maps roll/pitch/heading coordinate rates to FRD body angular rates.
local function attitudeCoordinateRatesToBodyRates(roll, pitch, rates)
    local sinRoll = math.sin(roll)
    local cosRoll = math.cos(roll)
    local sinPitch = math.sin(pitch)
    local cosPitch = math.cos(pitch)
    local rollRate = rates.roll or 0.0
    local pitchRate = rates.pitch or 0.0
    local headingRate = rates.heading or 0.0

    return {
        roll = rollRate - sinPitch * headingRate,
        pitch = cosRoll * pitchRate + sinRoll * cosPitch * headingRate,
        yaw = -sinRoll * pitchRate + cosRoll * cosPitch * headingRate,
    }
end

-- Inverse of attitudeCoordinateRatesToBodyRates; used for angle-loop D feedback.
local function bodyRatesToAttitudeCoordinateRates(roll, pitch, rates)
    local sinRoll = math.sin(roll)
    local cosRoll = math.cos(roll)
    local cosPitch = math.cos(pitch)
    local pitchYaw = sinRoll * (rates.pitch or 0.0) + cosRoll * (rates.yaw or 0.0)

    if math.abs(cosPitch) < 1.0e-4 then
        cosPitch = cosPitch >= 0.0 and 1.0e-4 or -1.0e-4
    end

    return {
        roll = (rates.roll or 0.0) + math.tan(pitch) * pitchYaw,
        pitch = cosRoll * (rates.pitch or 0.0) - sinRoll * (rates.yaw or 0.0),
        heading = pitchYaw / cosPitch,
    }
end

local function attitudeError(current, target)
    local worldError = (
        current.forward:cross(target.forward)
            + current.right:cross(target.right)
            + current.down:cross(target.down)
    ) * 0.5

    return mathx.project(worldError, {
        roll = current.forward,
        pitch = current.right,
        yaw = current.down,
    })
end

local function updateAngle(axis, bodyError, errorDerivative, dt)
    return axis.angle:update({
        target = bodyError,
        current = 0.0,
        error = bodyError,
        dt = dt,
        derivative = errorDerivative,
    })
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
    local bodyFrame = state.bodyFrame
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
    local currentBodyFrame = bodyFrame or bodyFrameFromPose(pose.roll, pose.pitch, pose.heading)

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
    local attitudeHeading = pose.heading
    local bodyRates = {
        roll = rollRate,
        pitch = pitchRate,
        yaw = yawRate,
    }
    local currentCoordinateRates = bodyRatesToAttitudeCoordinateRates(
        pose.roll,
        pose.pitch,
        bodyRates
    )
    local targetCoordinateRates = {
        roll = 0.0,
        pitch = 0.0,
        heading = headingTarget.rate or 0.0,
    }

    if headingActive then
        attitudeHeading = headingTarget.angle
    end

    local targetBodyFrame = bodyFrameFromPose(
        attitudeTarget.roll,
        attitudeTarget.pitch,
        attitudeHeading
    )
    local bodyAttitudeError = attitudeError(currentBodyFrame, targetBodyFrame)

    local rollAngleResult = updateAngle(
        pids.attitude.roll,
        bodyAttitudeError.roll,
        targetCoordinateRates.roll - currentCoordinateRates.roll,
        dt
    )
    local pitchAngleResult = updateAngle(
        pids.attitude.pitch,
        bodyAttitudeError.pitch,
        targetCoordinateRates.pitch - currentCoordinateRates.pitch,
        dt
    )
    local yawAngleResult = nil

    if headingActive then
        yawAngleResult = updateAngle(
            pids.attitude.yaw,
            bodyAttitudeError.yaw,
            targetCoordinateRates.heading - currentCoordinateRates.heading,
            dt
        )
    else
        pids.attitude.yaw.angle:reset()
    end

    local correctionCoordinateRates = {
        roll = rollAngleResult.output,
        pitch = pitchAngleResult.output,
        heading = yawAngleResult and yawAngleResult.output or 0.0,
    }
    local desiredCoordinateRates = {
        roll = targetCoordinateRates.roll + correctionCoordinateRates.roll,
        pitch = targetCoordinateRates.pitch + correctionCoordinateRates.pitch,
        heading = targetCoordinateRates.heading + correctionCoordinateRates.heading,
    }
    local targetBodyRates = attitudeCoordinateRatesToBodyRates(
        pose.roll,
        pose.pitch,
        desiredCoordinateRates
    )
    local rollRateResult = pids.attitude.roll.rate:update({
        target = targetBodyRates.roll,
        current = rollRate,
        dt = dt,
    })
    local pitchRateResult = pids.attitude.pitch.rate:update({
        target = targetBodyRates.pitch,
        current = pitchRate,
        dt = dt,
    })
    local yawRateResult = pids.attitude.yaw.rate:update({
        target = targetBodyRates.yaw,
        current = yawRate,
        dt = dt,
    })
    local rollResult = {
        angle = rollAngleResult,
        rate = rollRateResult,
    }
    local pitchResult = {
        angle = pitchAngleResult,
        rate = pitchRateResult,
    }

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
                },
                pitch = {
                    command = commands.pitch,
                    feedforward = pitchResult.rate.terms.ff,
                    feedback = pitchResult.rate.terms.raw,
                    targetRate = pitchResult.rate.target,
                },
                yaw = {
                    command = commands.yaw,
                    feedforward = yawRateResult.terms.ff,
                    feedback = yawRateResult.terms.raw,
                    targetRate = yawRateResult.target,
                },
                kinematics = {
                    currentCoordinateRates = currentCoordinateRates,
                    targetCoordinateRates = targetCoordinateRates,
                    correctionCoordinateRates = correctionCoordinateRates,
                    desiredCoordinateRates = desiredCoordinateRates,
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
                        output = desiredCoordinateRates.heading,
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
