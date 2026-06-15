local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

local function linearFeedforward(gain)
    gain = gain or 0.0

    return function(input)
        return gain * input.target
    end
end

local function collectiveFeedforward(collective, vertical)
    local bias = collective.feedforward_bias or 0.0
    local gain = vertical.speed_feedforward_gain or 0.0

    return function(input)
        return bias + gain * input.target
    end
end

local function vec(x, y, z)
    return {
        x = x,
        y = y,
        z = z,
    }
end

local function add(a, b)
    return vec(a.x + b.x, a.y + b.y, a.z + b.z)
end

local function scale(a, k)
    return vec(a.x * k, a.y * k, a.z * k)
end

local function cross(a, b)
    return vec(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    )
end

local function attitudeVerticalFactor(roll, pitch, minFactor)
    local factor = math.cos(roll) * math.cos(pitch)

    return mathx.clamp(factor, minFactor, 1.0)
end

local function frameFromPose(roll, pitch, heading)
    local sinHeading = math.sin(heading)
    local cosHeading = math.cos(heading)
    local sinPitch = math.sin(pitch)
    local cosPitch = math.cos(pitch)
    local sinRoll = math.sin(roll)
    local cosRoll = math.cos(roll)

    local forwardHorizontal = vec(sinHeading, 0.0, -cosHeading)
    local rightLevel = vec(cosHeading, 0.0, sinHeading)
    local worldDown = vec(0.0, -1.0, 0.0)
    local forward = add(
        scale(forwardHorizontal, cosPitch),
        scale(worldDown, -sinPitch)
    )
    local downLevel = cross(forward, rightLevel)

    return {
        forward = forward,
        right = add(
            scale(rightLevel, cosRoll),
            scale(downLevel, sinRoll)
        ),
        down = add(
            scale(rightLevel, -sinRoll),
            scale(downLevel, cosRoll)
        ),
    }
end

local function attitudeError(current, target)
    local worldError = scale(
        add(
            add(
                cross(current.forward, target.forward),
                cross(current.right, target.right)
            ),
            cross(current.down, target.down)
        ),
        0.5
    )

    return mathx.project(worldError, {
        roll = current.forward,
        pitch = current.right,
        yaw = current.down,
    })
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

    controllers.vertical.speed:setFeedforward(collectiveFeedforward(control.collective, control.vertical))
    controllers.attitude.roll.rate:setFeedforward(linearFeedforward(control.attitude.rate_feedforward.roll))
    controllers.attitude.pitch.rate:setFeedforward(linearFeedforward(control.attitude.rate_feedforward.pitch))
    controllers.attitude.yaw.rate:setFeedforward(linearFeedforward(control.attitude.rate_feedforward.yaw))

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
    local frame = state.frame
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
    local currentFrame = frame or frameFromPose(pose.roll, pose.pitch, pose.heading)

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

    local targetYawRate = headingTarget.rate
    local headingErr = headingTarget.error
    local headingActive = headingTarget.active
    local targetHeadingForAttitude = pose.heading

    if headingActive then
        targetHeadingForAttitude = headingTarget.angle
    end

    local targetFrame = frameFromPose(
        attitudeTarget.roll,
        attitudeTarget.pitch,
        targetHeadingForAttitude
    )
    local bodyAttitudeError = attitudeError(currentFrame, targetFrame)

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
    local yawAngleResult = nil

    if headingActive then
        yawAngleResult = pids.attitude.yaw.angle:update({
            target = bodyAttitudeError.yaw,
            current = 0.0,
            error = bodyAttitudeError.yaw,
            dt = dt,
            derivative = -yawRate,
        })
        targetYawRate = yawAngleResult.output
    else
        pids.attitude.yaw.angle:reset()
    end

    local yawRateResult = pids.attitude.yaw.rate:update({
        target = targetYawRate,
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
                        output = targetYawRate,
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
