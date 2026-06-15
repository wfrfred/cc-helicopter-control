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

local function horizontal(right, forward)
    return {
        right = right,
        forward = forward,
    }
end

local function emptyPositionState()
    return {
        target = horizontal(0.0, 0.0),
        current = horizontal(0.0, 0.0),
        error = horizontal(0.0, 0.0),
    }
end

local function makeInactiveResult()
    return {
        active = false,
        position = emptyPositionState(),
        velocity = {
            target = horizontal(0.0, 0.0),
            current = horizontal(0.0, 0.0),
            error = horizontal(0.0, 0.0),
        },
        output = {
            right = {
                value = 0.0,
                feedforward = 0.0,
                feedback = 0.0,
            },
            forward = {
                value = 0.0,
                feedforward = 0.0,
                feedback = 0.0,
            },
            attitude = {
                roll = nil,
                pitch = nil,
            },
        },
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
    local positionState = position or emptyPositionState()

    return {
        active = true,
        position = positionState,
        velocity = {
            target = targetVelocity,
            current = horizontalVelocity,
            error = horizontal(velocityErrorRight, velocityErrorForward),
        },
        output = {
            right = {
                value = outputRight,
                feedforward = feedforwardRight,
                feedback = feedbackRight,
            },
            forward = {
                value = outputForward,
                feedforward = feedforwardForward,
                feedback = feedbackForward,
            },
            attitude = {
                roll = mathx.clamp(outputRight, -self.control.attitude.limit.roll, self.control.attitude.limit.roll),
                pitch = mathx.clamp(outputForward, -self.control.attitude.limit.pitch, self.control.attitude.limit.pitch),
            },
        },
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
            target = horizontal(0.0, 0.0),
            current = horizontal(-bodyPositionError.right, -bodyPositionError.forward),
            error = horizontal(errorRight, errorForward),
        }
    )
end

function Hold:pidControllers()
    return self.controllers
end

return position_hold
