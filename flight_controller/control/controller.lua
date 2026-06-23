local allocation_control = require("control.allocation")
local attitude_control = require("control.attitude")
local horizontal_control = require("control.horizontal")
local vertical_control = require("control.vertical")

local controller = {}

local Controller = {}
Controller.__index = Controller

function controller.new(control)
    return setmetatable({
        horizontal = horizontal_control.new(control),
        vertical = vertical_control.new(control),
        attitude = attitude_control.new(control),
        allocation = allocation_control.new(control),
        lastTerms = {},
    }, Controller)
end

local function horizontalTarget(self, state, target, reset, dt)
    if reset.horizontal then
        self.horizontal:reset()
    end

    if target.world.position ~= nil then
        return self.horizontal:updatePosition(
            target.world.position,
            state.world.position,
            state.world.velocity,
            target.heading.angle,
            dt
        )
    end

    if target.world.velocity ~= nil then
        return self.horizontal:updateVelocity(
            target.world.velocity,
            state.world.velocity,
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

function Controller:update(input)
    local state = input.state
    local target = input.target
    local reset = input.reset or {}
    local horizontal = horizontalTarget(self, state, target, reset, input.dt)
    local vertical = self.vertical:update({
        state = state,
        target = target.vertical,
        dt = input.dt,
    })
    local attitudeCommands = self.attitude:update({
        state = state,
        commanded = desiredAttitude(target, horizontal),
        feedforward = target.attitude.feedforward,
        heading = target.heading.angle,
        headingError = target.heading.error,
        dt = input.dt,
    })
    local command = self.allocation:update({
        pose = state.body.pose,
        rawCommands = {
            collective = vertical.collective,
            roll = attitudeCommands.roll,
            pitch = attitudeCommands.pitch,
            yaw = attitudeCommands.yaw,
        },
    })

    self.lastTerms = {
        horizontal = self.horizontal:terms(),
        vertical = self.vertical:terms(),
        attitude = self.attitude:terms(),
        allocation = self.allocation:terms(),
    }

    return command
end

function Controller:terms()
    return self.lastTerms
end

return controller
