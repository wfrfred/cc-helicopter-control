local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

function controller.new(control)
    return setmetatable({
        control = control,

        heightPid = pid.new(control.pid.height),
        rollPid = pid.new(control.pid.roll),
        pitchPid = pid.new(control.pid.pitch),
        yawAnglePid = pid.new(control.pid.yaw_angle),
        yawRatePid = pid.new(control.pid.yaw_rate),
    }, Controller)
end

function Controller:update(input)
    local targets = input.targets
    local pose = input.pose
    local yawRate = input.yawRate
    local yawResult = input.yaw
    local dt = input.dt

    local heightOut, heightErr = self.heightPid:update(targets.height, pose.pos.y, dt)

    local rollErr = mathx.wrapPi(targets.roll - pose.roll)
    local rollCmd = self.rollPid:update(rollErr, 0.0, dt)

    local pitchErr = mathx.wrapPi(targets.pitch - pose.pitch)
    local pitchCmd = self.pitchPid:update(pitchErr, 0.0, dt)

    local targetYawRate = yawResult.commanded_rate
    local yawErr = yawResult.yaw_err
    local yawAngleActive = yawResult.angle_active

    if yawAngleActive then
        targetYawRate = self.yawAnglePid:update(yawErr, 0.0, dt)
    end

    local yawCmd, yawRateErr = self.yawRatePid:update(targetYawRate, yawRate, dt)

    local collective = mathx.clamp(
        self.control.base_collective + heightOut,
        self.control.collective_min,
        self.control.collective_max
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
        height = self.heightPid,
        roll = self.rollPid,
        pitch = self.pitchPid,
        yawAngle = self.yawAnglePid,
        yawRate = self.yawRatePid,
    }
end

return controller
