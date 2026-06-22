local feedforward = require("lib.feedforward")
local horizontal_control = require("control.horizontal")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local translation = {}

local Translation = {}
Translation.__index = Translation

local function attitudeVerticalFactor(roll, pitch, minFactor)
    local factor = math.cos(roll) * math.cos(pitch)

    return mathx.clamp(factor, minFactor, 1.0)
end

local function horizontalVelocity(state)
    return {
        x = state.world.velocity.x,
        z = state.world.velocity.z,
    }
end

local function horizontalPositionError(target, state)
    return {
        x = target.x - state.world.position.x,
        z = target.z - state.world.position.z,
    }
end

local function horizontalTarget(self, state, target, dt)
    if target.reset.horizontal then
        self.horizontal:reset()
    end

    if target.world.position ~= nil then
        return self.horizontal:updatePosition(
            horizontalPositionError(target.world.position, state),
            horizontalVelocity(state),
            target.heading.angle,
            dt
        )
    end

    if target.world.velocity ~= nil then
        return self.horizontal:updateVelocity(
            target.world.velocity,
            horizontalVelocity(state),
            target.heading.angle,
            dt
        )
    end

    return self.horizontal:inactive()
end

local function desiredAttitude(target, horizontalResult)
    if target.attitude.roll ~= nil and target.attitude.pitch ~= nil then
        return {
            roll = target.attitude.roll,
            pitch = target.attitude.pitch,
            source = target.source,
        }
    end

    return {
        roll = horizontalResult.output.attitude.roll,
        pitch = horizontalResult.output.attitude.pitch,
        source = target.source,
    }
end

function translation.new(control)
    local controllers = {
        height = pid.new(control.pid.vertical.height),
        speed = pid.new(control.pid.vertical.speed),
    }

    controllers.speed:setFeedforward(
        feedforward.linear(control.vertical.feedforward.gain, control.vertical.feedforward.bias)
    )

    return setmetatable({
        control = control,
        collective = control.collective,
        horizontal = horizontal_control.new(control),
        controllers = controllers,
        lastTerms = {},
    }, Translation)
end

function Translation:update(input)
    local state = input.state
    local target = input.target
    local dt = input.dt
    local pose = state.body.pose
    local verticalTarget = target.vertical
    local horizontalResult = horizontalTarget(self, state, target, dt)
    local attitude = desiredAttitude(target, horizontalResult)
    local targetVerticalSpeed = verticalTarget.speed
    local heightErr = verticalTarget.error

    if verticalTarget.active then
        local heightResult = self.controllers.height:update({
            target = verticalTarget.height,
            current = pose.height,
            dt = dt,
            derivative = -state.world.velocity.y,
        })
        targetVerticalSpeed = heightResult.output
        heightErr = heightResult.error
    else
        self.controllers.height:reset()
    end

    local verticalSpeedResult = self.controllers.speed:update({
        target = targetVerticalSpeed,
        current = state.world.velocity.y,
        dt = dt,
    })
    local collectiveOut = verticalSpeedResult.output
    local tiltVerticalFactor = attitudeVerticalFactor(
        pose.roll,
        pose.pitch,
        self.collective.tilt_compensation.min_factor
    )
    local tiltCompensation = 1.0 / tiltVerticalFactor
    local tiltCompensatedCollectiveOut = collectiveOut * tiltCompensation
    local collective = mathx.clamp(
        tiltCompensatedCollectiveOut,
        self.collective.min,
        self.collective.max
    )

    self.lastTerms = {
        horizontal = horizontalResult,
        vertical = {
            target = {
                height = verticalTarget.height,
                speed = targetVerticalSpeed,
                active = verticalTarget.active,
                pending = verticalTarget.pending,
            },
            current = {
                height = pose.height,
                speed = state.world.velocity.y,
            },
            error = {
                height = heightErr,
                speed = verticalSpeedResult.error,
            },
            terms = {
                height = self.controllers.height:terms(),
                speed = self.controllers.speed:terms(),
                tilt = {
                    compensation = tiltCompensation,
                    verticalFactor = tiltVerticalFactor,
                    uncompensated = collectiveOut,
                    output = tiltCompensatedCollectiveOut,
                },
            },
        },
    }

    return {
        collective = collective,
        attitude = attitude,
    }
end

function Translation:terms()
    return self.lastTerms
end

return translation
