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
        positionRight = pid.new(control.pid.position_right),
        positionForward = pid.new(control.pid.position_forward),
        velocityRight = pid.new(control.pid.velocity_right),
        velocityForward = pid.new(control.pid.velocity_forward),
    }

    return setmetatable({
        control = control,
        targetX = initial.pos.x,
        targetZ = initial.pos.z,
        active = false,
        velocityRightFeedforwardGain = control.position_hold_velocity_right_feedforward_gain,
        velocityForwardFeedforwardGain = control.position_hold_velocity_forward_feedforward_gain,
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
            errorRight = 0.0,
            errorForward = 0.0,
            targetVelocityRight = 0.0,
            targetVelocityForward = 0.0,
            currentVelocityRight = 0.0,
            currentVelocityForward = 0.0,
            feedforwardRight = 0.0,
            feedforwardForward = 0.0,
            feedbackRight = 0.0,
            feedbackForward = 0.0,
            outputRight = 0.0,
            outputForward = 0.0,
            roll = nil,
            pitch = nil,
        }
    end

    if not self.active then
        self.targetX = pose.pos.x
        self.targetZ = pose.pos.z
        self.active = true
    end

    local bodyPositionError = worldToBody(
        self.targetX - pose.pos.x,
        self.targetZ - pose.pos.z,
        pose.yaw
    )
    local bodyVelocity = worldToBody(velocity.x, velocity.z, pose.yaw)

    local targetVelocityRight, errorRight = self.controllers.positionRight:update(
        bodyPositionError.right,
        0.0,
        dt,
        -bodyVelocity.right
    )
    local targetVelocityForward, errorForward = self.controllers.positionForward:update(
        bodyPositionError.forward,
        0.0,
        dt,
        -bodyVelocity.forward
    )

    local feedforwardRight = self.velocityRightFeedforwardGain * targetVelocityRight
    local feedforwardForward = self.velocityForwardFeedforwardGain * targetVelocityForward
    local feedbackRight = self.controllers.velocityRight:update(targetVelocityRight, bodyVelocity.right, dt)
    local feedbackForward = self.controllers.velocityForward:update(targetVelocityForward, bodyVelocity.forward, dt)
    local outputRight = feedforwardRight + feedbackRight
    local outputForward = feedforwardForward + feedbackForward

    return {
        active = true,
        targetX = self.targetX,
        targetZ = self.targetZ,
        errorRight = errorRight,
        errorForward = errorForward,
        targetVelocityRight = targetVelocityRight,
        targetVelocityForward = targetVelocityForward,
        currentVelocityRight = bodyVelocity.right,
        currentVelocityForward = bodyVelocity.forward,
        feedforwardRight = feedforwardRight,
        feedforwardForward = feedforwardForward,
        feedbackRight = feedbackRight,
        feedbackForward = feedbackForward,
        outputRight = outputRight,
        outputForward = outputForward,
        roll = mathx.clamp(outputRight, -self.control.max_target_roll, self.control.max_target_roll),
        pitch = mathx.clamp(outputForward, -self.control.max_target_pitch, self.control.max_target_pitch),
    }
end

function Hold:pidControllers()
    return self.controllers
end

return position_hold
