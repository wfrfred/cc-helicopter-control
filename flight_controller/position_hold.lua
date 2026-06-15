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

local function linearFeedforward(gain)
    return function(input)
        return gain * input.target
    end
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

    controllers.velocityRight:setFeedforward(linearFeedforward(control.position_hold.velocity_feedforward.right))
    controllers.velocityForward:setFeedforward(linearFeedforward(control.position_hold.velocity_feedforward.forward))

    return setmetatable({
        control = control,
        controllers = controllers,
    }, Hold)
end

function Hold:reset()
    resetAll(self.controllers)
end

function Hold:updateVelocity(targetVelocity, horizontalVelocity, dt, position)
    local rightResult = self.controllers.velocityRight:update({
        target = targetVelocity.right,
        current = horizontalVelocity.right,
        dt = dt,
    })
    local forwardResult = self.controllers.velocityForward:update({
        target = targetVelocity.forward,
        current = horizontalVelocity.forward,
        dt = dt,
    })
    local outputRight = rightResult.output
    local outputForward = forwardResult.output
    local positionState = position or emptyPositionState()

    return {
        active = true,
        position = positionState,
        velocity = {
            target = targetVelocity,
            current = horizontalVelocity,
            error = horizontal(rightResult.error, forwardResult.error),
        },
        output = {
            right = {
                value = outputRight,
                feedforward = rightResult.terms.ff,
                feedback = rightResult.terms.raw,
            },
            forward = {
                value = outputForward,
                feedforward = forwardResult.terms.ff,
                feedback = forwardResult.terms.raw,
            },
            attitude = {
                roll = mathx.clamp(outputRight, -self.control.attitude.limit.roll, self.control.attitude.limit.roll),
                pitch = mathx.clamp(-outputForward, -self.control.attitude.limit.pitch, self.control.attitude.limit.pitch),
            },
        },
    }
end

function Hold:update(bodyPositionError, horizontalVelocity, dt)
    local rightResult = self.controllers.positionRight:update({
        target = bodyPositionError.right,
        current = 0.0,
        dt = dt,
        derivative = -horizontalVelocity.right,
    })
    local forwardResult = self.controllers.positionForward:update({
        target = bodyPositionError.forward,
        current = 0.0,
        dt = dt,
        derivative = -horizontalVelocity.forward,
    })

    return self:updateVelocity(
        {
            right = rightResult.output,
            forward = forwardResult.output,
        },
        horizontalVelocity,
        dt,
        {
            target = horizontal(0.0, 0.0),
            current = horizontal(-bodyPositionError.right, -bodyPositionError.forward),
            error = horizontal(rightResult.error, forwardResult.error),
        }
    )
end

function Hold:pidControllers()
    return self.controllers
end

return position_hold
