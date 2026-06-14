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

local function makeInactiveResult()
    return {
        active = false,
        targetRight = 0.0,
        targetForward = 0.0,
        currentPositionRight = 0.0,
        currentPositionForward = 0.0,
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
        velocityErrorRight = 0.0,
        velocityErrorForward = 0.0,
        roll = nil,
        pitch = nil,
    }
end

function position_hold.inactive()
    return makeInactiveResult()
end

function position_hold.new(control)
    local controllers = {
        positionRight = pid.new(control.pid.position.right),
        positionForward = pid.new(control.pid.position.forward),
        velocityRight = pid.new(control.pid.velocity.right),
        velocityForward = pid.new(control.pid.velocity.forward),
    }

    return setmetatable({
        control = control,
        velocityRightFeedforwardGain = control.position_hold.velocity_feedforward.right,
        velocityForwardFeedforwardGain = control.position_hold.velocity_feedforward.forward,
        controllers = controllers,
    }, Hold)
end

function Hold:reset()
    resetAll(self.controllers)
end

function Hold:updateVelocity(targetVelocity, horizontalVelocity, dt, position)
    local feedbackRight, velocityErrorRight = self.controllers.velocityRight:update(
        targetVelocity.right,
        horizontalVelocity.right,
        dt
    )
    local feedbackForward, velocityErrorForward = self.controllers.velocityForward:update(
        targetVelocity.forward,
        horizontalVelocity.forward,
        dt
    )
    local feedforwardRight = self.velocityRightFeedforwardGain * targetVelocity.right
    local feedforwardForward = self.velocityForwardFeedforwardGain * targetVelocity.forward
    local outputRight = feedforwardRight + feedbackRight
    local outputForward = feedforwardForward + feedbackForward
    local positionState = position or {
        current = {
            right = 0.0,
            forward = 0.0,
        },
        error = {
            right = 0.0,
            forward = 0.0,
        },
    }

    return {
        active = true,
        targetRight = 0.0,
        targetForward = 0.0,
        currentPositionRight = positionState.current.right,
        currentPositionForward = positionState.current.forward,
        errorRight = positionState.error.right,
        errorForward = positionState.error.forward,
        targetVelocityRight = targetVelocity.right,
        targetVelocityForward = targetVelocity.forward,
        currentVelocityRight = horizontalVelocity.right,
        currentVelocityForward = horizontalVelocity.forward,
        velocityErrorRight = velocityErrorRight,
        velocityErrorForward = velocityErrorForward,
        feedforwardRight = feedforwardRight,
        feedforwardForward = feedforwardForward,
        feedbackRight = feedbackRight,
        feedbackForward = feedbackForward,
        outputRight = outputRight,
        outputForward = outputForward,
        roll = mathx.clamp(outputRight, -self.control.attitude.limit.roll, self.control.attitude.limit.roll),
        pitch = mathx.clamp(outputForward, -self.control.attitude.limit.pitch, self.control.attitude.limit.pitch),
    }
end

function Hold:update(bodyPositionError, horizontalVelocity, dt)
    local targetVelocityRight, errorRight = self.controllers.positionRight:update(
        bodyPositionError.right,
        0.0,
        dt,
        -horizontalVelocity.right
    )
    local targetVelocityForward, errorForward = self.controllers.positionForward:update(
        bodyPositionError.forward,
        0.0,
        dt,
        -horizontalVelocity.forward
    )

    return self:updateVelocity(
        {
            right = targetVelocityRight,
            forward = targetVelocityForward,
        },
        horizontalVelocity,
        dt,
        {
            current = {
                right = -bodyPositionError.right,
                forward = -bodyPositionError.forward,
            },
            error = {
                right = errorRight,
                forward = errorForward,
            },
        }
    )
end

function Hold:pidControllers()
    return self.controllers
end

return position_hold
