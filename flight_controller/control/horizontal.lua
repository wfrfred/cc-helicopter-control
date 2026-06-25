local feedforward = require("lib.feedforward")
local mathx = require("lib.mathx")
local pid = require("lib.pid")
local tablex = require("lib.tablex")

local horizontal = {}

local Horizontal = {}
Horizontal.__index = Horizontal

function horizontal.new(control)
    local controllers = {
        positionForward = pid.new(control.pid.position.forward),
        positionRight = pid.new(control.pid.position.right),
        velocityForward = pid.new(control.pid.velocity.forward),
        velocityRight = pid.new(control.pid.velocity.right),
    }

    controllers.velocityForward:setFeedforward(
        feedforward.directionalLinear(
            control.position_hold.velocity_feedforward.forward.gain_neg,
            control.position_hold.velocity_feedforward.forward.gain_pos
        )
    )
    controllers.velocityRight:setFeedforward(
        feedforward.directionalLinear(
            control.position_hold.velocity_feedforward.right.gain_neg,
            control.position_hold.velocity_feedforward.right.gain_pos
        )
    )

    return setmetatable({
        control = control,
        controllers = controllers,
    }, Horizontal)
end

function Horizontal:reset()
    tablex.record.each(self.controllers, function(controller)
        controller:reset()
    end)
end

function Horizontal:update(state, target, feedforwardInput, dt)
    local currentPosition = state.position
    local currentVelocity = state.velocity
    local targetVelocity = {
        forward = feedforwardInput.position.forward,
        right = feedforwardInput.position.right,
    }
    local forwardResult = nil
    local rightResult = nil

    if target.position.forward ~= nil then
        forwardResult = self.controllers.positionForward:update({
            target = target.position.forward,
            current = currentPosition.forward,
            dt = dt,
            derivative = -currentVelocity.forward,
        })
        targetVelocity.forward = targetVelocity.forward + forwardResult.output
    else
        self.controllers.positionForward:reset()
    end

    if target.position.right ~= nil then
        rightResult = self.controllers.positionRight:update({
            target = target.position.right,
            current = currentPosition.right,
            dt = dt,
            derivative = -currentVelocity.right,
        })
        targetVelocity.right = targetVelocity.right + rightResult.output
    else
        self.controllers.positionRight:reset()
    end

    local forwardVelocityResult = self.controllers.velocityForward:update({
        target = targetVelocity.forward,
        current = currentVelocity.forward,
        dt = dt,
    })
    local rightVelocityResult = self.controllers.velocityRight:update({
        target = targetVelocity.right,
        current = currentVelocity.right,
        dt = dt,
    })
    local angle = {
        roll = mathx.clamp(
            rightVelocityResult.output + feedforwardInput.velocity.right,
            -self.control.attitude.limit.roll,
            self.control.attitude.limit.roll
        ),
        pitch = mathx.clamp(
            -(forwardVelocityResult.output + feedforwardInput.velocity.forward),
            -self.control.attitude.limit.pitch,
            self.control.attitude.limit.pitch
        ),
    }

    return {
        output = {
            angle = angle,
        },
        terms = {
            position = {
                target = tablex.record.copy(target.position),
                current = tablex.record.copy(currentPosition),
                error = {
                    forward = forwardResult and forwardResult.error or nil,
                    right = rightResult and rightResult.error or nil,
                },
            },
            velocity = {
                target = targetVelocity,
                current = tablex.record.copy(currentVelocity),
                error = {
                    forward = forwardVelocityResult.error,
                    right = rightVelocityResult.error,
                },
            },
            output = {
                angle = tablex.record.copy(angle),
            },
            pid = {
                position = {
                    forward = self.controllers.positionForward:terms(),
                    right = self.controllers.positionRight:terms(),
                },
                velocity = {
                    forward = self.controllers.velocityForward:terms(),
                    right = self.controllers.velocityRight:terms(),
                },
            },
        },
    }
end

return horizontal
