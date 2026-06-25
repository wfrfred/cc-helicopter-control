local mathx = require("lib.mathx")
local pid = require("lib.pid")
local tablex = require("lib.tablex")

local vertical = {}

local Vertical = {}
Vertical.__index = Vertical

function vertical.new(control)
    local controllers = {
        height = pid.new(control.pid.vertical.height),
        speed = pid.new(control.pid.vertical.speed),
    }

    return setmetatable({
        collective = control.collective,
        controllers = controllers,
        speedFeedforward = control.vertical.feedforward,
    }, Vertical)
end

function Vertical:reset()
    tablex.record.each(self.controllers, function(controller)
        controller:reset()
    end)
end

function Vertical:update(state, target, feedforwardInput, dt)
    local targetVelocity = feedforwardInput.position
    local positionResult = nil
    local positionTerms = nil

    if target.position ~= nil then
        positionResult = self.controllers.height:update(
            state.position,
            target.position,
            dt,
            state.velocity
        )

        positionTerms = positionResult.terms
        targetVelocity = targetVelocity + positionResult.output
    else
        positionTerms = self.controllers.height:reset()
    end

    local verticalSpeedResult = self.controllers.speed:update(state.velocity, targetVelocity, dt)
    local speedFeedforward = mathx.affine(
        targetVelocity,
        self.speedFeedforward.gain,
        self.speedFeedforward.bias
    )
    local collectiveOut = verticalSpeedResult.output + speedFeedforward + feedforwardInput.velocity
    local tiltVerticalFactor = mathx.clamp(
        -state.downAxis.y,
        self.collective.tilt_compensation.min_factor,
        1.0
    )
    local tiltCompensation = 1.0 / tiltVerticalFactor
    local tiltCompensatedCollectiveOut = collectiveOut * tiltCompensation
    local collective = mathx.clamp(
        tiltCompensatedCollectiveOut,
        self.collective.min,
        self.collective.max
    )

    return {
        output = {
            collective = collective,
        },
        terms = {
            position = {
                target = target.position,
                current = state.position,
                error = positionResult and positionResult.error or nil,
            },
            velocity = {
                target = targetVelocity,
                current = state.velocity,
                error = verticalSpeedResult.error,
            },
            output = {
                collective = collective,
            },
            collective = {
                raw = collectiveOut,
                tiltCompensated = tiltCompensatedCollectiveOut,
            },
            feedforward = {
                position = feedforwardInput.position,
                velocity = feedforwardInput.velocity,
            },
            tilt = {
                compensation = tiltCompensation,
                verticalFactor = tiltVerticalFactor,
            },
            pid = {
                position = positionTerms,
                velocity = verticalSpeedResult.terms,
            },
        },
    }
end

return vertical
