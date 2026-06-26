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

    return setmetatable({
        control = control,
        controllers = controllers,
        velocityFeedforward = control.position_hold.velocity_feedforward,
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
    local forwardPositionTerms = nil
    local rightPositionTerms = nil

    if target.position.forward ~= nil then
        forwardResult = self.controllers.positionForward:update(
            target.position.forward,
            currentPosition.forward,
            dt,
            currentVelocity.forward
        )
        forwardPositionTerms = forwardResult.terms
        targetVelocity.forward = targetVelocity.forward + forwardResult.output
    else
        forwardPositionTerms = self.controllers.positionForward:reset()
    end

    if target.position.right ~= nil then
        rightResult = self.controllers.positionRight:update(
            target.position.right,
            currentPosition.right,
            dt,
            currentVelocity.right
        )
        rightPositionTerms = rightResult.terms
        targetVelocity.right = targetVelocity.right + rightResult.output
    else
        rightPositionTerms = self.controllers.positionRight:reset()
    end

    local forwardVelocityResult = self.controllers.velocityForward:update(
        targetVelocity.forward,
        currentVelocity.forward,
        dt
    )
    local rightVelocityResult = self.controllers.velocityRight:update(
        targetVelocity.right,
        currentVelocity.right,
        dt
    )
    local forwardVelocityFeedforward = mathx.directionalAffine(
        targetVelocity.forward,
        self.velocityFeedforward.forward.gain_neg,
        self.velocityFeedforward.forward.gain_pos
    )
    local rightVelocityFeedforward = mathx.directionalAffine(
        targetVelocity.right,
        self.velocityFeedforward.right.gain_neg,
        self.velocityFeedforward.right.gain_pos
    )
    local roll = rightVelocityResult.output
        + rightVelocityFeedforward
        + feedforwardInput.velocity.right
    local pitch = -(forwardVelocityResult.output
        + forwardVelocityFeedforward
        + feedforwardInput.velocity.forward)
    local angle = {
        roll = mathx.clamp(
            roll,
            -self.control.attitude.limit.roll,
            self.control.attitude.limit.roll
        ),
        pitch = mathx.clamp(
            pitch,
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
                forward = {
                    target = target.position.forward,
                    current = currentPosition.forward,
                    error = forwardResult and forwardResult.terms.error or nil,
                    output = forwardResult and forwardResult.output or nil,
                    pid = forwardPositionTerms,
                },
                right = {
                    target = target.position.right,
                    current = currentPosition.right,
                    error = rightResult and rightResult.terms.error or nil,
                    output = rightResult and rightResult.output or nil,
                    pid = rightPositionTerms,
                },
            },
            velocity = {
                forward = {
                    target = targetVelocity.forward,
                    current = currentVelocity.forward,
                    error = forwardVelocityResult.terms.error,
                    output = forwardVelocityResult.output,
                    pid = forwardVelocityResult.terms,
                },
                right = {
                    target = targetVelocity.right,
                    current = currentVelocity.right,
                    error = rightVelocityResult.terms.error,
                    output = rightVelocityResult.output,
                    pid = rightVelocityResult.terms,
                },
            },
            output = {
                angle = tablex.record.copy(angle),
            },
            feedforward = {
                position = tablex.record.copy(feedforwardInput.position),
                velocity = tablex.record.copy(feedforwardInput.velocity),
                velocityTarget = {
                    forward = forwardVelocityFeedforward,
                    right = rightVelocityFeedforward,
                },
            },
        },
    }
end

return horizontal
