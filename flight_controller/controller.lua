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

local function updateRate(axisRatePid, targetRate, currentRate, dt)
    return axisRatePid:update({
        target = targetRate,
        current = currentRate,
        dt = dt,
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
                rate = pid.new(control.pid.attitude.roll.rate),
            },
            pitch = {
                rate = pid.new(control.pid.attitude.pitch.rate),
            },
            yaw = {
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
    local targetRates = attitude_math.bodyRateCommand(
        currentOrientation,
        attitudeTarget.orientation,
        self.attitude.time_constant
    )

    local rollRateResult = updateRate(
        pids.attitude.roll.rate,
        targetRates.roll,
        rollRate,
        dt
    )
    local pitchRateResult = updateRate(
        pids.attitude.pitch.rate,
        targetRates.pitch,
        pitchRate,
        dt
    )
    local yawRateResult = updateRate(
        pids.attitude.yaw.rate,
        targetRates.yaw,
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
        roll = rollRateResult.output,
        pitch = pitchRateResult.output,
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
                    feedforward = rollRateResult.terms.ff,
                    feedback = rollRateResult.terms.raw,
                    targetRate = targetRates.roll,
                    angleRate = targetRates.roll,
                },
                pitch = {
                    command = commands.pitch,
                    feedforward = pitchRateResult.terms.ff,
                    feedback = pitchRateResult.terms.raw,
                    targetRate = targetRates.pitch,
                    angleRate = targetRates.pitch,
                },
                yaw = {
                    command = commands.yaw,
                    feedforward = yawRateResult.terms.ff,
                    feedback = yawRateResult.terms.raw,
                    targetRate = targetRates.yaw,
                    angleRate = targetRates.yaw,
                },
            },
        },

        target = {
            vertical = {
                height = verticalTarget.height,
                speed = targetVerticalSpeed,
            },
            attitude = {
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
        },

        current = {
            vertical = {
                height = height,
                speed = verticalSpeed,
            },
            attitude = {
                roll = {
                    rate = rollRate,
                },
                pitch = {
                    rate = pitchRate,
                },
                yaw = {
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
                roll = {
                    rate = rollRateResult,
                    targetRate = targetRates.roll,
                    attitudeError = bodyAttitudeError.roll,
                },
                pitch = {
                    rate = pitchRateResult,
                    targetRate = targetRates.pitch,
                    attitudeError = bodyAttitudeError.pitch,
                },
                yaw = {
                    rate = yawRateResult,
                    targetRate = targetRates.yaw,
                    attitudeError = bodyAttitudeError.yaw,
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
