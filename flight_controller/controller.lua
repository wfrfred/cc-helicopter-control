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
        collectiveMin = control.collective_min,
        collectiveMax = control.collective_max,
        verticalSpeedFeedforwardGain = control.vertical_speed_feedforward_gain,
        verticalSpeedFeedforwardBias = control.vertical_speed_feedforward_bias,
        tiltCompensationMinFactor = control.tilt_compensation_min_factor,
        yawRateFeedforwardGain = control.yaw_rate_feedforward_gain,
        pitchFeedforwardBias = control.pitch_feedforward_bias,

        height = pid.new(control.pid.height),
        verticalSpeed = pid.new(control.pid.vertical_speed),
        roll = pid.new(control.pid.roll),
        pitch = pid.new(control.pid.pitch),
        yawAngle = pid.new(control.pid.yaw_angle),
        yawRate = pid.new(control.pid.yaw_rate),
    }, Controller)
end

function Controller:update(input)
    local targets = input.targets
    local pose = input.pose
    local velocity = input.velocity
    local heightResult = input.height
    local downSpeed = velocity.down
    local rollRate = input.rollRate
    local pitchRate = input.pitchRate
    local yawRate = input.yawRate
    local yawResult = input.yaw
    local dt = input.dt

    local targetDownSpeed = heightResult.commandedRate
    local heightErr = heightResult.error

    if heightResult.active then
        targetDownSpeed, heightErr = self.height:update(heightResult.target, pose.down, dt, -downSpeed)
    else
        self.height:reset()
    end

    local downSpeedFeedback, downSpeedErr = self.verticalSpeed:update(targetDownSpeed, downSpeed, dt)
    local downSpeedFeedforward = self.verticalSpeedFeedforwardGain * targetDownSpeed
    local collectiveFeedforward = self.verticalSpeedFeedforwardBias - downSpeedFeedforward
    local collectiveFeedback = -downSpeedFeedback
    local collectiveOut = collectiveFeedforward + collectiveFeedback
    local tiltVerticalFactor = attitudeVerticalFactor(
        pose.roll,
        pose.pitch,
        self.tiltCompensationMinFactor
    )
    local tiltCompensation = 1.0 / tiltVerticalFactor
    local tiltCompensatedCollectiveOut = collectiveOut * tiltCompensation

    local rollErr = mathx.wrapPi(targets.roll - pose.roll)
    local rollCmd = self.roll:update(rollErr, 0.0, dt, -rollRate)

    local pitchErr = mathx.wrapPi(targets.pitch - pose.pitch)
    local pitchFeedback = self.pitch:update(pitchErr, 0.0, dt, -pitchRate)
    local pitchFeedforward = self.pitchFeedforwardBias
    local pitchCmd = pitchFeedforward + pitchFeedback

    local targetYawRate = yawResult.commandedRate
    local yawErr = yawResult.error
    local yawAngleActive = yawResult.active

    if yawAngleActive then
        targetYawRate = self.yawAngle:update(yawErr, 0.0, dt, -yawRate)
    else
        self.yawAngle:reset()
    end

    local yawRateFeedback, yawRateErr = self.yawRate:update(targetYawRate, yawRate, dt)
    local yawRateFeedforward = self.yawRateFeedforwardGain * targetYawRate
    local yawCmd = yawRateFeedforward + yawRateFeedback

    local collective = mathx.clamp(
        tiltCompensatedCollectiveOut,
        self.collectiveMin,
        self.collectiveMax
    )

    return {
        commands = {
            collective = collective,
            roll = rollCmd,
            pitch = pitchCmd,
            yaw = yawCmd,
        },

        terms = {
            height = {
                target = heightResult.target,
                current = pose.down,
                err = heightErr,
                out = targetDownSpeed,
                lockActive = heightResult.active,
                lockPending = heightResult.pending,
            },

            verticalSpeed = {
                target = targetDownSpeed,
                current = downSpeed,
                err = downSpeedErr,
                feedforward = collectiveFeedforward,
                feedback = collectiveFeedback,
                controlFeedforward = downSpeedFeedforward,
                controlFeedback = downSpeedFeedback,
                tiltCompensation = tiltCompensation,
                tiltVerticalFactor = tiltVerticalFactor,
                uncompensatedOut = collectiveOut,
                out = tiltCompensatedCollectiveOut,
            },

            roll = {
                target = targets.roll,
                current = pose.roll,
                err = rollErr,
                rate = rollRate,
                out = rollCmd,
            },

            pitch = {
                target = targets.pitch,
                current = pose.pitch,
                err = pitchErr,
                rate = pitchRate,
                feedforward = pitchFeedforward,
                feedback = pitchFeedback,
                out = pitchCmd,
            },

            yaw = {
                target = yawResult.target,
                current = pose.yaw,
                err = yawErr,
                targetRate = targetYawRate,
                rate = yawRate,
                rateErr = yawRateErr,
                rateFeedforward = yawRateFeedforward,
                rateFeedback = yawRateFeedback,
                out = yawCmd,
                angleActive = yawAngleActive,
                anglePending = yawResult.pending,
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
