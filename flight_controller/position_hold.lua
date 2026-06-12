local mathx = require("lib.mathx")
local pid = require("lib.pid")

local position_hold = {}

local Hold = {}
Hold.__index = Hold

local function resetAll(controllers)
    for _, controller in pairs(controllers) do
        controller:reset()
    end
end

local function worldToBody(x, z, yaw)
    return {
        right = math.cos(yaw) * x + math.sin(yaw) * z,
        forward = math.sin(yaw) * x - math.cos(yaw) * z,
    }
end

function position_hold.new(initial, control)
    local controllers = {
        positionX = pid.new(control.pid.position_x),
        positionZ = pid.new(control.pid.position_z),
        velocityX = pid.new(control.pid.velocity_x),
        velocityZ = pid.new(control.pid.velocity_z),
    }

    return setmetatable({
        control = control,
        targetX = initial.pos.x,
        targetZ = initial.pos.z,
        active = false,
        velocityXFeedforwardGain = control.position_hold_velocity_x_feedforward_gain,
        velocityZFeedforwardGain = control.position_hold_velocity_z_feedforward_gain,
        controllers = controllers,
    }, Hold)
end

function Hold:update(input, pose, velocity, dt)
    local manual = input.roll ~= 0 or input.pitch ~= 0

    if manual then
        self.targetX = pose.pos.x
        self.targetZ = pose.pos.z
        self.active = false
        resetAll(self.controllers)

        return {
            active = false,
            targetX = self.targetX,
            targetZ = self.targetZ,
            errorX = 0.0,
            errorZ = 0.0,
            targetVelocityX = 0.0,
            targetVelocityZ = 0.0,
            feedforwardX = 0.0,
            feedforwardZ = 0.0,
            feedbackX = 0.0,
            feedbackZ = 0.0,
            outputX = 0.0,
            outputZ = 0.0,
            roll = nil,
            pitch = nil,
        }
    end

    if not self.active then
        self.targetX = pose.pos.x
        self.targetZ = pose.pos.z
        self.active = true
    end

    local targetVelocityX, errorX = self.controllers.positionX:update(self.targetX, pose.pos.x, dt)
    local targetVelocityZ, errorZ = self.controllers.positionZ:update(self.targetZ, pose.pos.z, dt)

    local feedforwardX = self.velocityXFeedforwardGain * targetVelocityX
    local feedforwardZ = self.velocityZFeedforwardGain * targetVelocityZ
    local feedbackX = self.controllers.velocityX:update(targetVelocityX, velocity.x, dt)
    local feedbackZ = self.controllers.velocityZ:update(targetVelocityZ, velocity.z, dt)
    local outputX = feedforwardX + feedbackX
    local outputZ = feedforwardZ + feedbackZ
    local body = worldToBody(outputX, outputZ, pose.yaw)

    return {
        active = true,
        targetX = self.targetX,
        targetZ = self.targetZ,
        errorX = errorX,
        errorZ = errorZ,
        targetVelocityX = targetVelocityX,
        targetVelocityZ = targetVelocityZ,
        feedforwardX = feedforwardX,
        feedforwardZ = feedforwardZ,
        feedbackX = feedbackX,
        feedbackZ = feedbackZ,
        outputX = outputX,
        outputZ = outputZ,
        roll = mathx.clamp(body.right, -self.control.max_target_roll, self.control.max_target_roll),
        pitch = mathx.clamp(body.forward, -self.control.max_target_pitch, self.control.max_target_pitch),
    }
end

function Hold:pidControllers()
    return self.controllers
end

return position_hold
