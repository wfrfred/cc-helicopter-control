local attitude_math = require("lib.attitude_math")
local feedforward = require("lib.feedforward")
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

    tablex.record.each(controllers, function(controller, axis)
        local rateFeedforward = control.attitude.rate_feedforward[axis]

        controller.rate:setFeedforward(
            feedforward.linear(rateFeedforward.gain, rateFeedforward.bias)
        )
    end)

    return setmetatable({
        controllers = controllers,
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
        return controller.angle:update({
            target = bodyAttitudeError[axis],
            current = 0.0,
            error = bodyAttitudeError[axis],
            derivative = -rates[axis],
            dt = dt,
        })
    end)
    local rateTargets = tablex.record.map(angleResults, function(result, axis)
        return result.output + angleFeedforward[axis]
    end)
    local rateResults = tablex.record.map(self.controllers, function(controller, axis)
        return controller.rate:update({
            target = rateTargets[axis],
            current = rates[axis],
            dt = dt,
        })
    end)
    local commands = tablex.record.map(rateResults, function(result, axis)
        return result.output + rateFeedforward[axis]
    end)

    return {
        output = commands,
        terms = {
            target = tablex.record.merge({
                orientation = target.orientation,
            }, tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = tablex.record.map(angleResults, function(result)
                    return result.target
                end),
                rate = rateTargets,
            })),
            current = tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = tablex.record.map(angleResults, function(result)
                    return result.current
                end),
                rate = tablex.record.pick(rates, { "roll", "pitch", "yaw" }),
            }),
            error = tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = tablex.record.map(angleResults, function(result)
                    return result.error
                end),
                rate = tablex.record.map(rateResults, function(result)
                    return result.error
                end),
            }),
            output = tablex.record.copy(commands),
            feedforward = {
                angle = tablex.record.copy(angleFeedforward),
                rate = tablex.record.copy(rateFeedforward),
            },
            pid = tablex.record.map(self.controllers, function(controller)
                return {
                    angle = controller.angle:terms(),
                    rate = controller.rate:terms(),
                }
            end),
        },
    }
end

return attitude
