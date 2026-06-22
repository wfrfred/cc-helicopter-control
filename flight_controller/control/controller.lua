local attitude_allocator = require("lib.attitude_allocator")
local attitude_math = require("lib.attitude_math")
local feedforward = require("lib.feedforward")
local horizontal_control = require("control.horizontal")
local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

local function attitudeVerticalFactor(roll, pitch, minFactor)
    local factor = math.cos(roll) * math.cos(pitch)

    return mathx.clamp(factor, minFactor, 1.0)
end

local function updateRate(axisRatePid, targetRate, currentRate, dt)
    return axisRatePid:update({
        target = targetRate,
        current = currentRate,
        dt = dt,
    })
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

local function copyCommands(commands)
    return {
        collective = commands.collective,
        roll = commands.roll,
        pitch = commands.pitch,
        yaw = commands.yaw,
    }
end

local function finalClampCommands(commands, limits)
    return {
        collective = commands.collective,
        roll = mathx.clamp(commands.roll, limits.roll_min, limits.roll_max),
        pitch = mathx.clamp(commands.pitch, limits.pitch_min, limits.pitch_max),
        yaw = mathx.clamp(commands.yaw, limits.yaw_min, limits.yaw_max),
    }
end

local function targetOrientation(control, currentFrame, attitude, heading)
    local fullFrame = attitude_math.frameFromPose(attitude.roll, attitude.pitch, heading)
    local full = attitude_math.quaternionFromFrame(fullFrame):normalize()
    local reducedFrame = attitude_math.reducedFrameFromTargetDown(currentFrame, fullFrame)
    local reduced = attitude_math.quaternionFromFrame(reducedFrame):normalize()
    local yawPriority = mathx.clamp(control.heading.yaw_priority, 0.0, 1.0)
    local mixed = reduced:slerp(full, yawPriority):normalize()

    return {
        roll = attitude.roll,
        pitch = attitude.pitch,
        orientation = mixed,
        fullOrientation = full,
        reducedOrientation = reduced,
        yawPriority = yawPriority,
    }
end

function controller.new(control)
    local controllers = {
        vertical = {
            height = pid.new(control.pid.vertical.height),
            speed = pid.new(control.pid.vertical.speed),
        },
        attitude = {
            roll = {
                rate = pid.new(control.pid.attitude.roll.rate),
            },
            pitch = {
                rate = pid.new(control.pid.attitude.pitch.rate),
            },
            yaw = {
                rate = pid.new(control.pid.attitude.yaw.rate),
            },
        },
    }

    controllers.vertical.speed:setFeedforward(
        feedforward.linear(control.vertical.feedforward.gain, control.vertical.feedforward.bias)
    )
    controllers.attitude.roll.rate:setFeedforward(
        feedforward.linear(
            control.attitude.rate_feedforward.roll.gain,
            control.attitude.rate_feedforward.roll.bias
        )
    )
    controllers.attitude.pitch.rate:setFeedforward(
        feedforward.linear(
            control.attitude.rate_feedforward.pitch.gain,
            control.attitude.rate_feedforward.pitch.bias
        )
    )
    controllers.attitude.yaw.rate:setFeedforward(
        feedforward.linear(control.attitude.rate_feedforward.yaw.gain)
    )

    return setmetatable({
        control = control,
        collective = control.collective,
        vertical = control.vertical,
        attitude = control.attitude,
        horizontal = horizontal_control.new(control),
        controllers = controllers,
        lastTerms = {},
    }, Controller)
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

local function desiredAttitude(self, state, target, horizontalResult)
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

function Controller:update(input)
    local target = input.target
    local state = input.state
    local pose = state.body.pose
    local rates = state.body.angular.velocity
    local verticalTarget = target.vertical
    local dt = input.dt
    local pids = self.controllers
    local horizontalResult = horizontalTarget(self, state, target, dt)
    local attitude = desiredAttitude(self, state, target, horizontalResult)
    local attitudeTarget = targetOrientation(
        self.control,
        state.body.frame,
        attitude,
        target.heading.angle
    )

    local targetVerticalSpeed = verticalTarget.speed
    local heightErr = verticalTarget.error
    local heightResult = nil

    if verticalTarget.active then
        heightResult = pids.vertical.height:update({
            target = verticalTarget.height,
            current = pose.height,
            dt = dt,
            derivative = -state.world.velocity.y,
        })
        targetVerticalSpeed = heightResult.output
        heightErr = heightResult.error
    else
        pids.vertical.height:reset()
    end

    local verticalSpeedResult = pids.vertical.speed:update({
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
    local bodyAttitudeError = attitude_math.attitudeError(
        state.body.orientation,
        attitudeTarget.orientation
    )
    local targetRates = attitude_math.bodyRateCommand(
        state.body.orientation,
        attitudeTarget.orientation,
        self.attitude.time_constant
    )
    local rollRateResult = updateRate(pids.attitude.roll.rate, targetRates.roll, rates.roll, dt)
    local pitchRateResult = updateRate(pids.attitude.pitch.rate, targetRates.pitch, rates.pitch, dt)
    local yawRateResult = updateRate(pids.attitude.yaw.rate, targetRates.yaw, rates.yaw, dt)
    local collective = mathx.clamp(
        tiltCompensatedCollectiveOut,
        self.collective.min,
        self.collective.max
    )
    local rawCommands = {
        collective = collective,
        roll = rollRateResult.output,
        pitch = pitchRateResult.output,
        yaw = yawRateResult.output,
    }
    local allocated = attitude_allocator.apply(self.control.attitude_allocator, pose, copyCommands(rawCommands))
    local commands = finalClampCommands(allocated.commands, self.control.output_limits)
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
                height = pids.vertical.height:terms(),
                speed = pids.vertical.speed:terms(),
                tilt = {
                    compensation = tiltCompensation,
                    verticalFactor = tiltVerticalFactor,
                    uncompensated = collectiveOut,
                    output = tiltCompensatedCollectiveOut,
                },
            },
        },
        attitude = {
            commanded = {
                roll = attitude.roll,
                pitch = attitude.pitch,
                heading = target.heading.angle,
                source = attitude.source,
            },
            target = {
                orientation = attitudeTarget.orientation,
                fullOrientation = attitudeTarget.fullOrientation,
                reducedOrientation = attitudeTarget.reducedOrientation,
                yawPriority = attitudeTarget.yawPriority,
                roll = {
                    rate = targetRates.roll,
                },
                pitch = {
                    rate = targetRates.pitch,
                },
                yaw = {
                    rate = targetRates.yaw,
                },
            },
            current = {
                roll = {
                    rate = rates.roll,
                },
                pitch = {
                    rate = rates.pitch,
                },
                yaw = {
                    rate = rates.yaw,
                },
                heading = {
                    angle = state.navigation.heading.angle,
                },
            },
            error = {
                roll = {
                    angle = bodyAttitudeError.roll,
                    rate = rollRateResult.error,
                },
                pitch = {
                    angle = bodyAttitudeError.pitch,
                    rate = pitchRateResult.error,
                },
                yaw = {
                    angle = bodyAttitudeError.yaw,
                    rate = yawRateResult.error,
                },
                heading = {
                    angle = target.heading.error,
                },
            },
            terms = {
                roll = {
                    rate = pids.attitude.roll.rate:terms(),
                },
                pitch = {
                    rate = pids.attitude.pitch.rate:terms(),
                },
                yaw = {
                    rate = pids.attitude.yaw.rate:terms(),
                },
            },
        },
        allocation = {
            rawCommands = rawCommands,
            allocatedCommands = allocated.commands,
            finalCommands = commands,
            debug = allocated.debug,
        },
    }

    return commands
end

function Controller:terms()
    return self.lastTerms
end

return controller
