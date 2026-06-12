local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

function controller.new(control)
    return setmetatable({
        collectiveMin = control.collective_min,
        collectiveMax = control.collective_max,
        verticalSpeedFeedforwardGain = control.vertical_speed_feedforward_gain,
        verticalSpeedFeedforwardBias = control.vertical_speed_feedforward_bias,

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
    local yawRate = input.yawRate
    local yawResult = input.yaw
    local dt = input.dt

    local targetVerticalSpeed = heightResult.commandedRate
    local heightErr = heightResult.error

    if heightResult.active then
        targetVerticalSpeed, heightErr = self.height:update(heightResult.target, pose.pos.y, dt)
    else
        self.height:reset()
    end

    local verticalSpeedFeedback, verticalSpeedErr = self.verticalSpeed:update(targetVerticalSpeed, velocity.vertical, dt)
    local verticalSpeedFeedforward = self.verticalSpeedFeedforwardGain * targetVerticalSpeed
        + self.verticalSpeedFeedforwardBias
    local verticalSpeedOut = verticalSpeedFeedforward + verticalSpeedFeedback

    local rollErr = mathx.wrapPi(targets.roll - pose.roll)
    local rollCmd = self.roll:update(rollErr, 0.0, dt)

    local pitchErr = mathx.wrapPi(targets.pitch - pose.pitch)
    local pitchCmd = self.pitch:update(pitchErr, 0.0, dt)

    local targetYawRate = yawResult.commandedRate
    local yawErr = yawResult.error
    local yawAngleActive = yawResult.active

    if yawAngleActive then
        targetYawRate = self.yawAngle:update(yawErr, 0.0, dt)
    end

    local yawCmd, yawRateErr = self.yawRate:update(targetYawRate, yawRate, dt)

    local collective = mathx.clamp(
        verticalSpeedOut,
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
            },

            verticalSpeed = {
                target = targetVerticalSpeed,
                current = velocity.vertical,
                err = verticalSpeedErr,
                feedforward = verticalSpeedFeedforward,
                feedback = verticalSpeedFeedback,
                out = verticalSpeedOut,
            },

            roll = {
                target = targets.roll,
                current = pose.roll,
                err = rollErr,
                out = rollCmd,
            },

            pitch = {
                target = targets.pitch,
                current = pose.pitch,
                err = pitchErr,
                out = pitchCmd,
            },

            yaw = {
                target = yawResult.target,
                current = pose.yaw,
                err = yawErr,
                targetRate = targetYawRate,
                rate = yawRate,
                rateErr = yawRateErr,
                out = yawCmd,
                angleActive = yawAngleActive,
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
