local feedforward = require("lib.feedforward")
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

local function worldHorizontal(x, z)
    return {
        x = x,
        z = z,
    }
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
        right = {
            x = math.cos(heading),
            z = math.sin(heading),
        },
        forward = {
            x = math.sin(heading),
            z = -math.cos(heading),
        },
    }
end

local function projectWorldHorizontalToNavigation(value, heading)
    return mathx.project(value, navigationHorizontalAxes(heading))
end

local function projectNavigationHorizontalToWorld(value, heading)
    local axes = navigationHorizontalAxes(heading)

    return worldHorizontal(
        value.right * axes.right.x + value.forward * axes.forward.x,
        value.right * axes.right.z + value.forward * axes.forward.z
    )
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
                roll = nil,
                pitch = nil,
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

function position_hold.inactive()
    return makeInactiveResult()
end

function position_hold.new(control)
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
    }, Hold)
end

function Hold:reset()
    resetAll(self.controllers)
end

local function updateNavigationVelocity(
    self,
    targetNavigationVelocity,
    currentNavigationVelocity,
    targetWorldVelocity,
    worldVelocity,
    attitudeHeading,
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
    local worldTilt = projectNavigationHorizontalToWorld(navigationTilt, attitudeHeading)
    local attitude = attitudeFromNavigationTilt(navigationTilt, self.control.attitude.limit)

    return {
        active = true,
        worldPosition = worldPosition or emptyWorldState(),
        navigationPosition = navigationPosition or emptyNavigationState(),
        worldVelocity = {
            target = targetWorldVelocity,
            current = worldVelocity,
            error = worldHorizontal(
                targetWorldVelocity.x - worldVelocity.x,
                targetWorldVelocity.z - worldVelocity.z
            ),
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
    }
end

function Hold:updateVelocity(targetWorldVelocity, worldVelocity, attitudeHeading, dt, position)
    local targetNavigationVelocity = projectWorldHorizontalToNavigation(
        targetWorldVelocity,
        attitudeHeading
    )
    local currentNavigationVelocity = projectWorldHorizontalToNavigation(worldVelocity, attitudeHeading)

    return updateNavigationVelocity(
        self,
        targetNavigationVelocity,
        currentNavigationVelocity,
        targetWorldVelocity,
        worldVelocity,
        attitudeHeading,
        dt,
        position,
        nil
    )
end

function Hold:update(worldPositionError, worldVelocity, attitudeHeading, dt)
    local navigationPositionError = projectWorldHorizontalToNavigation(
        worldPositionError,
        attitudeHeading
    )
    local currentNavigationVelocity = projectWorldHorizontalToNavigation(worldVelocity, attitudeHeading)
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
        attitudeHeading
    )

    return updateNavigationVelocity(
        self,
        targetNavigationVelocity,
        currentNavigationVelocity,
        targetWorldVelocity,
        worldVelocity,
        attitudeHeading,
        dt,
        {
            target = worldHorizontal(0.0, 0.0),
            current = worldHorizontal(-worldPositionError.x, -worldPositionError.z),
            error = worldPositionError,
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

return position_hold
