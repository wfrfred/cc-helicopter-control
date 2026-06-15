local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

local function attitudeVerticalFactor(roll, pitch, minFactor)
    local factor = math.cos(roll) * math.cos(pitch)

    return mathx.clamp(factor, minFactor, 1.0)
end

function controller.new(control)
    return setmetatable({
        collective = control.collective,
        vertical = control.vertical,
        attitude = control.attitude,
        yaw = control.yaw,

        height = pid.new(control.pid.vertical.height),
        verticalSpeed = pid.new(control.pid.vertical.speed),
        roll = pid.new(control.pid.attitude.roll),
        pitch = pid.new(control.pid.attitude.pitch),
        yawAngle = pid.new(control.pid.yaw.angle),
        yawRate = pid.new(control.pid.yaw.rate),
    }, Controller)
end

function Controller:update(input)
    local target = input.target
    local state = input.state
    local pose = state.pose
    local rates = state.rates
    local vertical = state.vertical
    local attitudeTarget = target.attitude
    local verticalTarget = target.vertical
    local yawTarget = target.yaw
    local height = vertical.height
    local verticalSpeed = vertical.speed
    local rollRate = rates.roll
    local pitchRate = rates.pitch
    local yawRate = rates.yaw
    local dt = input.dt

    local targetVerticalSpeed = verticalTarget.speed
    local heightErr = verticalTarget.error

    if verticalTarget.active then
        targetVerticalSpeed, heightErr = self.height:update(
            verticalTarget.height,
            height,
            dt,
            -verticalSpeed
        )
    else
        self.height:reset()
    end

    local verticalSpeedFeedback, verticalSpeedErr = self.verticalSpeed:update(
        targetVerticalSpeed,
        verticalSpeed,
        dt
    )
    local verticalSpeedFeedforward = self.vertical.speed_feedforward_gain * targetVerticalSpeed
    local collectiveFeedforward = self.collective.feedforward_bias + verticalSpeedFeedforward
    local collectiveFeedback = verticalSpeedFeedback
    local collectiveOut = collectiveFeedforward + collectiveFeedback
    local tiltVerticalFactor = attitudeVerticalFactor(
        pose.roll,
        pose.pitch,
        self.collective.tilt_compensation.min_factor
    )
    local tiltCompensation = 1.0 / tiltVerticalFactor
    local tiltCompensatedCollectiveOut = collectiveOut * tiltCompensation

    local rollErr = mathx.wrapPi(attitudeTarget.roll - pose.roll)
    local rollCmd = self.roll:update(rollErr, 0.0, dt, -rollRate)

    local pitchErr = mathx.wrapPi(attitudeTarget.pitch - pose.pitch)
    local pitchFeedback = self.pitch:update(pitchErr, 0.0, dt, -pitchRate)
    local pitchFeedforward = self.attitude.pitch.feedforward_bias
    local pitchCmd = pitchFeedforward + pitchFeedback

    local targetYawRate = yawTarget.rate
    local yawErr = yawTarget.error
    local yawAngleActive = yawTarget.active

    if yawAngleActive then
        targetYawRate = self.yawAngle:update(yawErr, 0.0, dt, -yawRate)
    else
        self.yawAngle:reset()
    end

    local yawRateFeedback, yawRateErr = self.yawRate:update(targetYawRate, yawRate, dt)
    local yawRateFeedforward = self.yaw.rate_feedforward_gain * targetYawRate
    local yawCmd = yawRateFeedforward + yawRateFeedback

    local collective = mathx.clamp(
        tiltCompensatedCollectiveOut,
        self.collective.min,
        self.collective.max
    )

    local commands = {
        collective = collective,
        roll = rollCmd,
        pitch = pitchCmd,
        yaw = yawCmd,
    }

    return {
        commands = commands,

        output = {
            commands = commands,
            collective = {
                command = commands.collective,
                feedforward = collectiveFeedforward,
                feedback = collectiveFeedback,
                uncompensated = collectiveOut,
                tilt = {
                    compensation = tiltCompensation,
                    verticalFactor = tiltVerticalFactor,
                },
            },
            pitch = {
                command = commands.pitch,
                feedforward = pitchFeedforward,
                feedback = pitchFeedback,
            },
            yaw = {
                command = commands.yaw,
                feedforward = yawRateFeedforward,
                feedback = yawRateFeedback,
            },
        },

        target = {
            vertical = {
                height = verticalTarget.height,
                speed = targetVerticalSpeed,
            },
            attitude = {
                roll = attitudeTarget.roll,
                pitch = attitudeTarget.pitch,
            },
            yaw = {
                angle = yawTarget.angle,
                rate = targetYawRate,
            },
        },

        current = {
            vertical = {
                height = height,
                speed = verticalSpeed,
            },
            attitude = {
                roll = pose.roll,
                pitch = pose.pitch,
            },
            yaw = {
                angle = pose.yaw,
                rate = yawRate,
            },
        },

        error = {
            vertical = {
                height = heightErr,
                speed = verticalSpeedErr,
            },
            attitude = {
                roll = rollErr,
                pitch = pitchErr,
            },
            yaw = {
                angle = yawErr,
                rate = yawRateErr,
            },
        },

        terms = {
            height = {
                target = verticalTarget.height,
                current = height,
                err = heightErr,
                out = targetVerticalSpeed,
                lockActive = verticalTarget.active,
                lockPending = verticalTarget.pending,
            },

            verticalSpeed = {
                target = targetVerticalSpeed,
                current = verticalSpeed,
                err = verticalSpeedErr,
                feedforward = collectiveFeedforward,
                feedback = collectiveFeedback,
                controlFeedforward = verticalSpeedFeedforward,
                controlFeedback = verticalSpeedFeedback,
                tiltCompensation = tiltCompensation,
                tiltVerticalFactor = tiltVerticalFactor,
                uncompensatedOut = collectiveOut,
                out = tiltCompensatedCollectiveOut,
            },

            roll = {
                target = attitudeTarget.roll,
                current = pose.roll,
                err = rollErr,
                rate = rollRate,
                out = rollCmd,
            },

            pitch = {
                target = attitudeTarget.pitch,
                current = pose.pitch,
                err = pitchErr,
                rate = pitchRate,
                feedforward = pitchFeedforward,
                feedback = pitchFeedback,
                out = pitchCmd,
            },

            yaw = {
                target = yawTarget.angle,
                current = pose.yaw,
                err = yawErr,
                targetRate = targetYawRate,
                rate = yawRate,
                rateErr = yawRateErr,
                rateFeedforward = yawRateFeedforward,
                rateFeedback = yawRateFeedback,
                out = yawCmd,
                angleActive = yawAngleActive,
                anglePending = yawTarget.pending,
            },
        },
    }
end

function Controller:pidControllers()
    return {
        height = self.height,
        verticalSpeed = self.verticalSpeed,
        roll = self.roll,
        pitch = self.pitch,
        yawAngle = self.yawAngle,
        yawRate = self.yawRate,
    }
end

return controller
