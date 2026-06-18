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

local function updateAngleRate(axis, bodyError, currentRate, dt)
    local angle = axis.angle:update({
        target = bodyError,
        current = 0.0,
        error = bodyError,
        dt = dt,
        derivative = -currentRate,
    })
    local rate = axis.rate:update({
        target = angle.output,
        current = currentRate,
        dt = dt,
    })

    return {
        angle = angle,
        rate = rate,
        correctionRate = angle.output,
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

    local bodyAttitudeError = attitude_math.attitudeError(
        currentOrientation,
        attitudeTarget.orientation
    )

    local rollResult = updateAngleRate(
        pids.attitude.roll,
        bodyAttitudeError.roll,
        rollRate,
        dt
    )
    local pitchResult = updateAngleRate(
        pids.attitude.pitch,
        bodyAttitudeError.pitch,
        pitchRate,
        dt
    )
    local yawResult = updateAngleRate(
        pids.attitude.yaw,
        bodyAttitudeError.yaw,
        yawRate,
        dt
    )

    local collective = mathx.clamp(
        tiltCompensatedCollectiveOut,
        self.collective.min,
        self.collective.max
    )

    local commands = {
        collective = collective,
        roll = rollResult.rate.output,
        pitch = pitchResult.rate.output,
        yaw = yawResult.rate.output,
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
                },
                pitch = {
                    command = commands.pitch,
                    feedforward = pitchResult.rate.terms.ff,
                    feedback = pitchResult.rate.terms.raw,
                    targetRate = pitchResult.rate.target,
                    angleRate = pitchResult.correctionRate,
                },
                yaw = {
                    command = commands.yaw,
                    feedforward = yawResult.rate.terms.ff,
                    feedback = yawResult.rate.terms.raw,
                    targetRate = yawResult.rate.target,
                    angleRate = yawResult.correctionRate,
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
                    angle = yawResult.angle.target,
                    rate = yawResult.rate.target,
                },
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
                    angle = yawResult.angle.current,
                    rate = yawRate,
                },
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
                    rate = yawResult.rate.error,
                },
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
                yaw = yawResult,
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
