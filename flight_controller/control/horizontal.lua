local feedforward = require("lib.feedforward")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local horizontal = {}

local Hold = {}
Hold.__index = Hold

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

local function navigationHorizontal(forward, right)
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

local function emptyNavigationState()
    return {
        target = navigationHorizontal(0.0, 0.0),
        current = navigationHorizontal(0.0, 0.0),
        error = navigationHorizontal(0.0, 0.0),
    }
end

local function navigationHorizontalAxes(heading)
    return {
        right = vector.new(math.cos(heading), 0.0, math.sin(heading)),
        forward = vector.new(math.sin(heading), 0.0, -math.cos(heading)),
    }
end

local function projectWorldHorizontalToNavigation(value, heading)
    local horizontal = horizontalVector(value)
    local axes = navigationHorizontalAxes(heading)

    return navigationHorizontal(
        horizontal:dot(axes.forward),
        horizontal:dot(axes.right)
    )
end

local function projectNavigationHorizontalToWorld(value, heading)
    local axes = navigationHorizontalAxes(heading)

    return axes.right * value.right + axes.forward * value.forward
end

local function makeInactiveResult()
    return {
        active = false,
        worldPosition = emptyWorldState(),
        navigationPosition = emptyNavigationState(),
        worldVelocity = emptyWorldState(),
        navigationVelocity = emptyNavigationState(),
        output = {
            worldTilt = {
                x = {
                    value = 0.0,
                },
                z = {
                    value = 0.0,
                },
            },
            navigationTilt = {
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

local function attitudeFromNavigationTilt(navigationTilt, limit)
    return {
        roll = mathx.clamp(navigationTilt.right, -limit.roll, limit.roll),
        pitch = mathx.clamp(-navigationTilt.forward, -limit.pitch, limit.pitch),
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
    }, Hold)
end

function Hold:reset()
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

function Hold:inactive()
    return attachTerms(self, makeInactiveResult())
end

local function updateNavigationVelocity(
    self,
    targetNavigationVelocity,
    currentNavigationVelocity,
    targetWorldVelocity,
    worldVelocity,
    heading,
    dt,
    worldPosition,
    navigationPosition
)
    local forwardResult = self.controllers.velocityForward:update({
        target = targetNavigationVelocity.forward,
        current = currentNavigationVelocity.forward,
        dt = dt,
    })
    local rightResult = self.controllers.velocityRight:update({
        target = targetNavigationVelocity.right,
        current = currentNavigationVelocity.right,
        dt = dt,
    })
    local navigationTilt = navigationHorizontal(forwardResult.output, rightResult.output)
    local worldTilt = projectNavigationHorizontalToWorld(navigationTilt, heading)
    local attitude = attitudeFromNavigationTilt(navigationTilt, self.control.attitude.limit)
    local worldVelocityError = targetWorldVelocity - worldVelocity

    return attachTerms(self, {
        active = true,
        worldPosition = worldPosition or emptyWorldState(),
        navigationPosition = navigationPosition or emptyNavigationState(),
        worldVelocity = {
            target = worldHorizontalFromVector(targetWorldVelocity),
            current = worldHorizontalFromVector(worldVelocity),
            error = worldHorizontalFromVector(worldVelocityError),
        },
        navigationVelocity = {
            target = targetNavigationVelocity,
            current = currentNavigationVelocity,
            error = navigationHorizontal(forwardResult.error, rightResult.error),
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
            navigationTilt = {
                forward = axisOutput(forwardResult),
                right = axisOutput(rightResult),
            },
            attitude = attitude,
        },
    })
end

function Hold:updateVelocity(targetWorldVelocity, worldVelocity, heading, dt, position)
    local targetWorld = horizontalVector(targetWorldVelocity)
    local currentWorld = horizontalVector(worldVelocity)
    local targetNavigationVelocity = projectWorldHorizontalToNavigation(
        targetWorld,
        heading
    )
    local currentNavigationVelocity = projectWorldHorizontalToNavigation(currentWorld, heading)

    return updateNavigationVelocity(
        self,
        targetNavigationVelocity,
        currentNavigationVelocity,
        targetWorld,
        currentWorld,
        heading,
        dt,
        position,
        nil
    )
end

function Hold:updateTranslation(position, feedforward, worldVelocity, heading, dt)
    local currentWorldVelocity = horizontalVector(worldVelocity)
    local currentNavigationVelocity = projectWorldHorizontalToNavigation(currentWorldVelocity, heading)
    local forwardResult = nil
    local rightResult = nil
    local targetNavigationVelocity = {
        forward = feedforward.forward,
        right = feedforward.right,
    }
    local worldPosition = emptyWorldState()
    local navigationPosition = emptyNavigationState()

    if position.forward ~= nil then
        forwardResult = self.controllers.positionForward:update({
            target = position.forward,
            current = 0.0,
            dt = dt,
            derivative = -currentNavigationVelocity.forward,
        })
        targetNavigationVelocity.forward = targetNavigationVelocity.forward + forwardResult.output
        navigationPosition.target.forward = 0.0
        navigationPosition.current.forward = -position.forward
        navigationPosition.error.forward = forwardResult.error
    end

    if position.right ~= nil then
        rightResult = self.controllers.positionRight:update({
            target = position.right,
            current = 0.0,
            dt = dt,
            derivative = -currentNavigationVelocity.right,
        })
        targetNavigationVelocity.right = targetNavigationVelocity.right + rightResult.output
        navigationPosition.target.right = 0.0
        navigationPosition.current.right = -position.right
        navigationPosition.error.right = rightResult.error
    end

    return updateNavigationVelocity(
        self,
        targetNavigationVelocity,
        currentNavigationVelocity,
        projectNavigationHorizontalToWorld(targetNavigationVelocity, heading),
        currentWorldVelocity,
        heading,
        dt,
        worldPosition,
        navigationPosition
    )
end

function Hold:updatePosition(targetWorldPosition, currentWorldPosition, worldVelocity, heading, dt)
    local worldPositionError = horizontalVector(targetWorldPosition) - horizontalVector(currentWorldPosition)
    local currentWorldVelocity = horizontalVector(worldVelocity)
    local navigationPositionError = projectWorldHorizontalToNavigation(
        worldPositionError,
        heading
    )
    local currentNavigationVelocity = projectWorldHorizontalToNavigation(currentWorldVelocity, heading)
    local forwardResult = self.controllers.positionForward:update({
        target = navigationPositionError.forward,
        current = 0.0,
        dt = dt,
        derivative = -currentNavigationVelocity.forward,
    })
    local rightResult = self.controllers.positionRight:update({
        target = navigationPositionError.right,
        current = 0.0,
        dt = dt,
        derivative = -currentNavigationVelocity.right,
    })
    local targetNavigationVelocity = navigationHorizontal(forwardResult.output, rightResult.output)
    local targetWorldVelocity = projectNavigationHorizontalToWorld(
        targetNavigationVelocity,
        heading
    )

    return updateNavigationVelocity(
        self,
        targetNavigationVelocity,
        currentNavigationVelocity,
        targetWorldVelocity,
        currentWorldVelocity,
        heading,
        dt,
        {
            target = worldHorizontal(0.0, 0.0),
            current = worldHorizontalFromVector(-worldPositionError),
            error = worldHorizontalFromVector(worldPositionError),
        },
        {
            target = navigationHorizontal(0.0, 0.0),
            current = navigationHorizontal(
                -navigationPositionError.forward,
                -navigationPositionError.right
            ),
            error = navigationHorizontal(forwardResult.error, rightResult.error),
        }
    )
end

function Hold:pidControllers()
    return self.controllers
end

function Hold:terms()
    return self.lastTerms
end

return horizontal
