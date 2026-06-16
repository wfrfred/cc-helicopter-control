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

local function emptyPositionState()
    return {
        target = worldHorizontal(0.0, 0.0),
        current = worldHorizontal(0.0, 0.0),
        error = worldHorizontal(0.0, 0.0),
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

local function makeInactiveResult()
    return {
        active = false,
        worldPosition = emptyPositionState(),
        worldVelocity = {
            target = worldHorizontal(0.0, 0.0),
            current = worldHorizontal(0.0, 0.0),
            error = worldHorizontal(0.0, 0.0),
        },
        output = {
            worldTilt = {
                x = {
                    value = 0.0,
                    feedforward = 0.0,
                    feedback = 0.0,
                },
                z = {
                    value = 0.0,
                    feedforward = 0.0,
                    feedback = 0.0,
                },
            },
            navigationTilt = {
                right = 0.0,
                forward = 0.0,
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

local function attitudeFromWorldTilt(worldTilt, attitudeHeading, limit)
    local navigationTilt = projectWorldHorizontalToNavigation(worldTilt, attitudeHeading)

    return navigationTilt, {
        roll = mathx.clamp(navigationTilt.right, -limit.roll, limit.roll),
        pitch = mathx.clamp(-navigationTilt.forward, -limit.pitch, limit.pitch),
    }
end

function position_hold.inactive()
    return makeInactiveResult()
end

function position_hold.new(control)
    local controllers = {
        positionX = pid.new(control.pid.position.x),
        positionZ = pid.new(control.pid.position.z),
        velocityX = pid.new(control.pid.velocity.x),
        velocityZ = pid.new(control.pid.velocity.z),
    }

    controllers.velocityX:setFeedforward(
        feedforward.linear(control.position_hold.velocity_feedforward.x)
    )
    controllers.velocityZ:setFeedforward(
        feedforward.linear(control.position_hold.velocity_feedforward.z)
    )

    return setmetatable({
        control = control,
        controllers = controllers,
    }, Hold)
end

function Hold:reset()
    resetAll(self.controllers)
end

function Hold:updateVelocity(targetWorldVelocity, worldVelocity, attitudeHeading, dt, position)
    local xResult = self.controllers.velocityX:update({
        target = targetWorldVelocity.x,
        current = worldVelocity.x,
        dt = dt,
    })
    local zResult = self.controllers.velocityZ:update({
        target = targetWorldVelocity.z,
        current = worldVelocity.z,
        dt = dt,
    })
    local worldTilt = {
        x = xResult.output,
        z = zResult.output,
    }
    local navigationTilt, attitude = attitudeFromWorldTilt(
        worldTilt,
        attitudeHeading,
        self.control.attitude.limit
    )
    local positionState = position or emptyPositionState()

    return {
        active = true,
        worldPosition = positionState,
        worldVelocity = {
            target = targetWorldVelocity,
            current = worldVelocity,
            error = worldHorizontal(xResult.error, zResult.error),
        },
        output = {
            worldTilt = {
                x = axisOutput(xResult),
                z = axisOutput(zResult),
            },
            navigationTilt = navigationTilt,
            attitude = attitude,
        },
    }
end

function Hold:update(worldPositionError, worldVelocity, attitudeHeading, dt)
    local xResult = self.controllers.positionX:update({
        target = worldPositionError.x,
        current = 0.0,
        dt = dt,
        derivative = -worldVelocity.x,
    })
    local zResult = self.controllers.positionZ:update({
        target = worldPositionError.z,
        current = 0.0,
        dt = dt,
        derivative = -worldVelocity.z,
    })

    return self:updateVelocity(
        {
            x = xResult.output,
            z = zResult.output,
        },
        worldVelocity,
        attitudeHeading,
        dt,
        {
            target = worldHorizontal(0.0, 0.0),
            current = worldHorizontal(-worldPositionError.x, -worldPositionError.z),
            error = worldHorizontal(xResult.error, zResult.error),
        }
    )
end

function Hold:pidControllers()
    return self.controllers
end

return position_hold
