local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

function controller.new(control)
    return setmetatable({
        baseCollective = control.base_collective,
        collectiveMin = control.collective_min,
        collectiveMax = control.collective_max,

        height = pid.new(control.pid.height),
        roll = pid.new(control.pid.roll),
        pitch = pid.new(control.pid.pitch),
        yawAngle = pid.new(control.pid.yaw_angle),
        yawRate = pid.new(control.pid.yaw_rate),
    }, Controller)
end

function Controller:update(input)
    local targets = input.targets
    local pose = input.pose
    local yawRate = input.yawRate
    local yawResult = input.yaw
    local dt = input.dt

    local heightOut, heightErr = self.height:update(targets.height, pose.pos.y, dt)

    local rollErr = mathx.wrapPi(targets.roll - pose.roll)
    local rollCmd = self.roll:update(rollErr, 0.0, dt)

    local pitchErr = mathx.wrapPi(targets.pitch - pose.pitch)
    local pitchCmd = self.pitch:update(pitchErr, 0.0, dt)

    local targetYawRate = yawResult.commanded_rate
    local yawErr = yawResult.yaw_err
    local yawAngleActive = yawResult.angle_active

    if yawAngleActive then
        targetYawRate = self.yawAngle:update(yawErr, 0.0, dt)
    end

    local yawCmd, yawRateErr = self.yawRate:update(targetYawRate, yawRate, dt)

    local collective = mathx.clamp(
        self.baseCollective + heightOut,
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
                target = targets.height,
                current = pose.pos.y,
                err = heightErr,
                out = heightOut,
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
                target = yawResult.target_yaw,
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
        roll = self.roll,
        pitch = self.pitch,
        yawAngle = self.yawAngle,
        yawRate = self.yawRate,
    }
end

return controller
