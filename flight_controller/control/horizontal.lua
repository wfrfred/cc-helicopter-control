local feedforward = require("lib.feedforward")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local horizontal = {}

local Horizontal = {}
Horizontal.__index = Horizontal

local function resetAll(controllers)
    for _, controller in pairs(controllers) do
        controller:reset()
    end
end

local function worldHorizontal(x, z)
    return {
        x = x,
        z = z,
    }
end

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

local function worldHorizontalFromVector(value)
    return worldHorizontal(value.x, value.z)
end

local function frameHorizontal(forward, right)
    return {
        forward = forward,
        right = right,
    }
end

local function emptyWorldState()
    return {
        target = worldHorizontal(0.0, 0.0),
        current = worldHorizontal(0.0, 0.0),
        error = worldHorizontal(0.0, 0.0),
    }
end

local function emptyFrameState()
    return {
        target = frameHorizontal(0.0, 0.0),
        current = frameHorizontal(0.0, 0.0),
        error = frameHorizontal(0.0, 0.0),
    }
end

local function projectWorldHorizontalToFrame(value, frame)
    local horizontal = horizontalVector(value)

    return frameHorizontal(
        horizontal:dot(frame.forward),
        horizontal:dot(frame.right)
    )
end

local function projectFrameHorizontalToWorld(value, frame)
    return frame.right * value.right + frame.forward * value.forward
end

local function makeInactiveResult()
    return {
        active = false,
        worldPosition = emptyWorldState(),
        framePosition = emptyFrameState(),
        worldVelocity = emptyWorldState(),
        frameVelocity = emptyFrameState(),
        output = {
            worldTilt = {
                x = {
                    value = 0.0,
                },
                z = {
                    value = 0.0,
                },
            },
            frameTilt = {
                forward = {
                    value = 0.0,
                    feedforward = 0.0,
                    feedback = 0.0,
                },
                right = {
                    value = 0.0,
                    feedforward = 0.0,
                    feedback = 0.0,
                },
            },
            attitude = {
                roll = 0.0,
                pitch = 0.0,
            },
        },
    }
end

local function axisOutput(result)
    return {
        value = result.output,
        feedforward = result.terms.ff,
        feedback = result.terms.raw,
    }
end

local function attitudeFromFrameTilt(frameTilt, limit)
    return {
        roll = mathx.clamp(frameTilt.right, -limit.roll, limit.roll),
        pitch = mathx.clamp(-frameTilt.forward, -limit.pitch, limit.pitch),
    }
end

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
        lastTerms = makeInactiveResult(),
    }, Horizontal)
end

function Horizontal:reset()
    resetAll(self.controllers)
end

local function attachTerms(self, result)
    result.terms = {
        position = {
            forward = self.controllers.positionForward:terms(),
            right = self.controllers.positionRight:terms(),
        },
        velocity = {
            forward = self.controllers.velocityForward:terms(),
            right = self.controllers.velocityRight:terms(),
        },
    }
    self.lastTerms = result

    return result
end

function Horizontal:inactive()
    return attachTerms(self, makeInactiveResult())
end

local function updateFrameVelocity(
    self,
    targetFrameVelocity,
    currentFrameVelocity,
    targetWorldVelocity,
    worldVelocity,
    frame,
    dt,
    worldPosition,
    framePosition
)
    local forwardResult = self.controllers.velocityForward:update({
        target = targetFrameVelocity.forward,
        current = currentFrameVelocity.forward,
        dt = dt,
    })
    local rightResult = self.controllers.velocityRight:update({
        target = targetFrameVelocity.right,
        current = currentFrameVelocity.right,
        dt = dt,
    })
    local frameTilt = frameHorizontal(forwardResult.output, rightResult.output)
    local worldTilt = projectFrameHorizontalToWorld(frameTilt, frame)
    local attitude = attitudeFromFrameTilt(frameTilt, self.control.attitude.limit)
    local worldVelocityError = targetWorldVelocity - worldVelocity

    return attachTerms(self, {
        active = true,
        worldPosition = worldPosition or emptyWorldState(),
        framePosition = framePosition or emptyFrameState(),
        worldVelocity = {
            target = worldHorizontalFromVector(targetWorldVelocity),
            current = worldHorizontalFromVector(worldVelocity),
            error = worldHorizontalFromVector(worldVelocityError),
        },
        frameVelocity = {
            target = targetFrameVelocity,
            current = currentFrameVelocity,
            error = frameHorizontal(forwardResult.error, rightResult.error),
        },
        output = {
            worldTilt = {
                x = {
                    value = worldTilt.x,
                },
                z = {
                    value = worldTilt.z,
                },
            },
            frameTilt = {
                forward = axisOutput(forwardResult),
                right = axisOutput(rightResult),
            },
            attitude = attitude,
        },
    })
end

function Horizontal:update(input)
    local state = input.state
    local frame = input.frame
    local target = input.target
    local feedforward = input.feedforward
    local dt = input.dt
    local position = target.position
    local velocity = feedforward.velocity
    local currentWorldVelocity = horizontalVector(state.world.velocity)
    local currentFrameVelocity = projectWorldHorizontalToFrame(currentWorldVelocity, frame)
    local forwardResult = nil
    local rightResult = nil
    local targetFrameVelocity = {
        forward = velocity.forward,
        right = velocity.right,
    }
    local worldPosition = emptyWorldState()
    local framePosition = emptyFrameState()

    if position.forward ~= nil then
        forwardResult = self.controllers.positionForward:update({
            target = position.forward,
            current = 0.0,
            dt = dt,
            derivative = -currentFrameVelocity.forward,
        })
        targetFrameVelocity.forward = targetFrameVelocity.forward + forwardResult.output
        framePosition.target.forward = 0.0
        framePosition.current.forward = -position.forward
        framePosition.error.forward = forwardResult.error
    end

    if position.right ~= nil then
        rightResult = self.controllers.positionRight:update({
            target = position.right,
            current = 0.0,
            dt = dt,
            derivative = -currentFrameVelocity.right,
        })
        targetFrameVelocity.right = targetFrameVelocity.right + rightResult.output
        framePosition.target.right = 0.0
        framePosition.current.right = -position.right
        framePosition.error.right = rightResult.error
    end

    return updateFrameVelocity(
        self,
        targetFrameVelocity,
        currentFrameVelocity,
        projectFrameHorizontalToWorld(targetFrameVelocity, frame),
        currentWorldVelocity,
        frame,
        dt,
        worldPosition,
        framePosition
    )
end

function Horizontal:terms()
    return self.lastTerms
end

return horizontal
