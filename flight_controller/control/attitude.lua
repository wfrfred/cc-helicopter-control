local attitude_math = require("lib.attitude_math")
local feedforward = require("lib.feedforward")
local pid = require("lib.pid")
local tablex = require("lib.tablex")

local attitude = {}

local Attitude = {}
Attitude.__index = Attitude

local function axisTable(fn)
    return tablex.reduce({ "roll", "pitch", "yaw" }, function(out, axis)
        out[axis] = fn(axis)

        return out
    end, {})
end

local function updateAngle(axisAnglePid, targetAngle, currentRate, dt)
    return axisAnglePid:update({
        target = targetAngle,
        current = 0.0,
        error = targetAngle,
        derivative = -currentRate,
        dt = dt,
    })
end

local function updateRate(axisRatePid, targetRate, currentRate, dt)
    return axisRatePid:update({
        target = targetRate,
        current = currentRate,
        dt = dt,
    })
end

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

    controllers.roll.rate:setFeedforward(
        feedforward.linear(
            control.attitude.rate_feedforward.roll.gain,
            control.attitude.rate_feedforward.roll.bias
        )
    )
    controllers.pitch.rate:setFeedforward(
        feedforward.linear(
            control.attitude.rate_feedforward.pitch.gain,
            control.attitude.rate_feedforward.pitch.bias
        )
    )
    controllers.yaw.rate:setFeedforward(
        feedforward.linear(control.attitude.rate_feedforward.yaw.gain)
    )

    return setmetatable({
        control = control,
        controllers = controllers,
        lastTerms = {},
    }, Attitude)
end

function Attitude:update(input)
    local state = input.state
    local target = input.target
    local externalFeedforward = input.feedforward
    local dt = input.dt
    local rates = state.body.angular.velocity
    local angleFeedforward = externalFeedforward.angle
    local rateFeedforward = externalFeedforward.rate
    local bodyAttitudeError = attitude_math.attitudeError(
        state.body.orientation,
        target.orientation
    )
    local angleResults = axisTable(function(axis)
        return updateAngle(
            self.controllers[axis].angle,
            bodyAttitudeError[axis],
            rates[axis],
            dt
        )
    end)
    local rateTargets = axisTable(function(axis)
        return angleResults[axis].output + angleFeedforward[axis]
    end)
    local rateResults = axisTable(function(axis)
        return updateRate(
            self.controllers[axis].rate,
            rateTargets[axis],
            rates[axis],
            dt
        )
    end)
    local commands = axisTable(function(axis)
        return rateResults[axis].output + rateFeedforward[axis]
    end)
    local angleTargets = axisTable(function(axis)
        return angleResults[axis].target
    end)
    local angleCurrent = axisTable(function(axis)
        return angleResults[axis].current
    end)
    local angleErrors = axisTable(function(axis)
        return angleResults[axis].error
    end)
    local rateErrors = axisTable(function(axis)
        return rateResults[axis].error
    end)
    local controllerTerms = axisTable(function(axis)
        return {
            angle = self.controllers[axis].angle:terms(),
            rate = self.controllers[axis].rate:terms(),
        }
    end)

    self.lastTerms = {
        target = tablex.merge({
            orientation = target.orientation,
        }, tablex.transpose({ "roll", "pitch", "yaw" }, {
            angle = angleTargets,
            rate = rateTargets,
        })),
        current = tablex.transpose({ "roll", "pitch", "yaw" }, {
            angle = angleCurrent,
            rate = tablex.pick(rates, { "roll", "pitch", "yaw" }),
        }),
        error = tablex.transpose({ "roll", "pitch", "yaw" }, {
            angle = angleErrors,
            rate = rateErrors,
        }),
        terms = controllerTerms,
        feedforward = {
            angle = tablex.copy(angleFeedforward),
            rate = tablex.copy(rateFeedforward),
        },
    }

    return commands
end

function Attitude:terms()
    return self.lastTerms
end

return attitude
