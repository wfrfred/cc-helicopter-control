local allocation_control = require("control.allocation")
local attitude_control = require("control.attitude")
local horizontal_control = require("control.horizontal")
local mathx = require("lib.mathx")
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

local function updateHorizontal(self, state, target, heading, reset, dt)
    local translation = target.translation
    local position = translation.position
    local feedforward = translation.feedforward

    if reset.horizontal then
        self.horizontal:reset()
    end

    if position.forward ~= nil
        or position.right ~= nil
        or feedforward.forward ~= 0.0
        or feedforward.right ~= 0.0 then
        return self.horizontal:updateTranslation(
            position,
            feedforward,
            state.world.velocity,
            heading,
            dt
        )
    end

    return self.horizontal:inactive()
end

local function desiredAttitude(target, horizontalResult, heading)
    local angle = target.attitude.angle

    return {
        roll = angle.roll or horizontalResult.output.attitude.roll,
        pitch = angle.pitch or horizontalResult.output.attitude.pitch,
        yaw = angle.yaw or heading,
    }
end

function Controller:update(input)
    local state = input.state
    local target = input.target
    local reset = input.reset or {}
    local attitudeAngle = target.attitude.angle
    local heading = attitudeAngle.yaw or state.navigation.heading.angle
    local horizontal = updateHorizontal(self, state, target, heading, reset, input.dt)
    local height = nil

    if target.translation.position.down ~= nil then
        height = state.body.pose.height - target.translation.position.down
    end

    local vertical = self.vertical:update({
        state = state,
        target = {
            height = height,
            velocity = -target.translation.feedforward.down,
        },
        dt = input.dt,
    })
    local attitudeCommands = self.attitude:update({
        state = state,
        commanded = desiredAttitude(target, horizontal, heading),
        feedforward = target.attitude.feedforward,
        heading = heading,
        headingError = attitudeAngle.yaw == nil
            and 0.0
            or mathx.wrapPi(attitudeAngle.yaw - state.navigation.heading.angle),
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
