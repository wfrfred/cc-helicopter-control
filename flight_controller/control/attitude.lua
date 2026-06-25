local attitude_math = require("lib.attitude_math")
local feedforward = require("lib.feedforward")
local pid = require("lib.pid")
local tablex = require("lib.tablex")

local attitude = {}

local Attitude = {}
Attitude.__index = Attitude

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
        controllers = controllers,
    }, Attitude)
end

function Attitude:reset()
    tablex.record.each(self.controllers, function(axis)
        axis.angle:reset()
        axis.rate:reset()
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
    local angleResults = {
        roll = updateAngle(self.controllers.roll.angle, bodyAttitudeError.roll, rates.roll, dt),
        pitch = updateAngle(self.controllers.pitch.angle, bodyAttitudeError.pitch, rates.pitch, dt),
        yaw = updateAngle(self.controllers.yaw.angle, bodyAttitudeError.yaw, rates.yaw, dt),
    }
    local rateTargets = {
        roll = angleResults.roll.output + angleFeedforward.roll,
        pitch = angleResults.pitch.output + angleFeedforward.pitch,
        yaw = angleResults.yaw.output + angleFeedforward.yaw,
    }
    local rateResults = {
        roll = updateRate(self.controllers.roll.rate, rateTargets.roll, rates.roll, dt),
        pitch = updateRate(self.controllers.pitch.rate, rateTargets.pitch, rates.pitch, dt),
        yaw = updateRate(self.controllers.yaw.rate, rateTargets.yaw, rates.yaw, dt),
    }
    local commands = {
        roll = rateResults.roll.output + rateFeedforward.roll,
        pitch = rateResults.pitch.output + rateFeedforward.pitch,
        yaw = rateResults.yaw.output + rateFeedforward.yaw,
    }

    return {
        output = commands,
        terms = {
            target = tablex.record.merge({
                orientation = target.orientation,
            }, tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = {
                    roll = angleResults.roll.target,
                    pitch = angleResults.pitch.target,
                    yaw = angleResults.yaw.target,
                },
                rate = rateTargets,
            })),
            current = tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = {
                    roll = angleResults.roll.current,
                    pitch = angleResults.pitch.current,
                    yaw = angleResults.yaw.current,
                },
                rate = tablex.record.pick(rates, { "roll", "pitch", "yaw" }),
            }),
            error = tablex.record.transpose({ "roll", "pitch", "yaw" }, {
                angle = {
                    roll = angleResults.roll.error,
                    pitch = angleResults.pitch.error,
                    yaw = angleResults.yaw.error,
                },
                rate = {
                    roll = rateResults.roll.error,
                    pitch = rateResults.pitch.error,
                    yaw = rateResults.yaw.error,
                },
            }),
            output = tablex.record.copy(commands),
            feedforward = {
                angle = tablex.record.copy(angleFeedforward),
                rate = tablex.record.copy(rateFeedforward),
            },
            pid = {
                roll = {
                    angle = self.controllers.roll.angle:terms(),
                    rate = self.controllers.roll.rate:terms(),
                },
                pitch = {
                    angle = self.controllers.pitch.angle:terms(),
                    rate = self.controllers.pitch.rate:terms(),
                },
                yaw = {
                    angle = self.controllers.yaw.angle:terms(),
                    rate = self.controllers.yaw.rate:terms(),
                },
            },
        },
    }
end

return attitude
