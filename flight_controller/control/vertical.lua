local feedforward = require("lib.feedforward")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local vertical = {}

local Vertical = {}
Vertical.__index = Vertical

local function attitudeVerticalFactor(roll, pitch, minFactor)
    local factor = math.cos(roll) * math.cos(pitch)

    return mathx.clamp(factor, minFactor, 1.0)
end

function vertical.new(control)
    local controllers = {
        height = pid.new(control.pid.vertical.height),
        speed = pid.new(control.pid.vertical.speed),
    }

    controllers.speed:setFeedforward(
        feedforward.linear(control.vertical.feedforward.gain, control.vertical.feedforward.bias)
    )

    return setmetatable({
        collective = control.collective,
        controllers = controllers,
        lastTerms = {},
    }, Vertical)
end

function Vertical:update(input)
    local state = input.state
    local target = input.target
    local dt = input.dt
    local pose = state.body.pose
    local targetVerticalSpeed = target.velocity
    local heightErr = target.height == nil and 0.0 or target.height - pose.height

    if target.height ~= nil then
        local heightResult = self.controllers.height:update({
            target = target.height,
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
        target = {
            height = target.height,
            speed = targetVerticalSpeed,
            active = target.height ~= nil,
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
    }

    return {
        collective = collective,
    }
end

function Vertical:terms()
    return self.lastTerms
end

return vertical
