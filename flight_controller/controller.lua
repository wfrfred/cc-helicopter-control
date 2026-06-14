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
        tiltCompensationMinFactor = control.tilt_compensation_min_factor or 0.5,
        yawRateFeedforwardGain = control.yaw_rate_feedforward_gain or 0.0,

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
    local rollRate = input.rollRate or 0.0
    local pitchRate = input.pitchRate or 0.0
    local yawRate = input.yawRate or 0.0
    local yawResult = input.yaw
    local dt = input.dt

    local targetVerticalSpeed = heightResult.commandedRate
    local heightErr = heightResult.error

    if heightResult.active then
        targetVerticalSpeed, heightErr = self.height:update(heightResult.target, pose.pos.y, dt, -velocity.vertical)
    else
        self.height:reset()
    end

    local verticalSpeedFeedback, verticalSpeedErr = self.verticalSpeed:update(targetVerticalSpeed, velocity.vertical, dt)
    local verticalSpeedFeedforward = self.verticalSpeedFeedforwardGain * targetVerticalSpeed
        + self.verticalSpeedFeedforwardBias
    local verticalSpeedOut = verticalSpeedFeedforward + verticalSpeedFeedback
    local tiltVerticalFactor = attitudeVerticalFactor(
        pose.roll,
        pose.pitch,
        self.tiltCompensationMinFactor
    )
    local tiltCompensation = 1.0 / tiltVerticalFactor
    local tiltCompensatedVerticalSpeedOut = verticalSpeedOut * tiltCompensation

    local rollErr = mathx.wrapPi(targets.roll - pose.roll)
    local rollCmd = self.roll:update(rollErr, 0.0, dt, -rollRate)

    local pitchErr = mathx.wrapPi(targets.pitch - pose.pitch)
    local pitchCmd = self.pitch:update(pitchErr, 0.0, dt, -pitchRate)

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
        tiltCompensatedVerticalSpeedOut,
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
                current = pose.pos.y,
                err = heightErr,
                out = targetVerticalSpeed,
                lockActive = heightResult.active,
                lockPending = heightResult.pending,
            },

            verticalSpeed = {
                target = targetVerticalSpeed,
                current = velocity.vertical,
                err = verticalSpeedErr,
                feedforward = verticalSpeedFeedforward,
                feedback = verticalSpeedFeedback,
                tiltCompensation = tiltCompensation,
                tiltVerticalFactor = tiltVerticalFactor,
                uncompensatedOut = verticalSpeedOut,
                out = tiltCompensatedVerticalSpeedOut,
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
