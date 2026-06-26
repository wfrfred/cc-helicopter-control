local attitude_math = require("lib.attitude_math")
local mathx = require("lib.mathx")
local pid = require("lib.pid")
local tablex = require("lib.tablex")

local attitude = {}

local Attitude = {}
Attitude.__index = Attitude

function attitude.new(control)
    local controllers = {
        roll = {
            angle = pid.new(control.pid.attitude.roll.angle),
            rate = pid.new(control.pid.attitude.roll.rate),
        },
        pitch = {
            angle = pid.new(control.pid.attitude.pitch.angle),
            rate = pid.new(control.pid.attitude.pitch.rate),
        },
        yaw = {
            angle = pid.new(control.pid.attitude.yaw.angle),
            rate = pid.new(control.pid.attitude.yaw.rate),
        },
    }

    return setmetatable({
        controllers = controllers,
        rateFeedforward = control.attitude.rate_feedforward,
    }, Attitude)
end

function Attitude:reset()
    tablex.record.each(self.controllers, function(controller)
        controller.angle:reset()
        controller.rate:reset()
    end)
end

function Attitude:update(state, target, feedforwardInput, dt)
    local rates = state.angularVelocity
    local angleFeedforward = feedforwardInput.angle
    local rateFeedforward = feedforwardInput.rate
    local bodyAttitudeError = attitude_math.attitudeError(
        state.orientation,
        target.orientation
    )
    local angleResults = tablex.record.map(self.controllers, function(controller, axis)
        return controller.angle:update(bodyAttitudeError[axis], 0.0, dt, rates[axis])
    end)
    local rateTargets = tablex.record.map(angleResults, function(result, axis)
        return result.output + angleFeedforward[axis]
    end)
    local rateResults = tablex.record.map(self.controllers, function(controller, axis)
        return controller.rate:update(rateTargets[axis], rates[axis], dt)
    end)
    local rateTargetFeedforward = tablex.record.map(rateTargets, function(targetRate, axis)
        local config = self.rateFeedforward[axis]

        return mathx.affine(targetRate, config.gain, config.bias)
    end)
    local commands = tablex.record.map(rateResults, function(result, axis)
        return result.output + rateTargetFeedforward[axis] + rateFeedforward[axis]
    end)

    return {
        output = commands,
        terms = {
            target = tablex.record.merge({
                orientation = target.orientation,
            }, tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = tablex.record.map(angleResults, function(result)
                    return result.terms.target
                end),
                rate = rateTargets,
            })),
            current = tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = tablex.record.map(angleResults, function(result)
                    return result.terms.current
                end),
                rate = tablex.record.pick(rates, { "roll", "pitch", "yaw" }),
            }),
            error = tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = tablex.record.map(angleResults, function(result)
                    return result.terms.error
                end),
                rate = tablex.record.map(rateResults, function(result)
                    return result.terms.error
                end),
            }),
            output = tablex.record.copy(commands),
            feedforward = {
                angle = tablex.record.copy(angleFeedforward),
                rate = tablex.record.copy(rateFeedforward),
                rateTarget = tablex.record.copy(rateTargetFeedforward),
            },
            pid = tablex.record.map(self.controllers, function(_controller, axis)
                return {
                    angle = angleResults[axis].terms,
                    rate = rateResults[axis].terms,
                }
            end),
        },
    }
end

return attitude
