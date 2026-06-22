local allocation_control = require("control.allocation")
local attitude_control = require("control.attitude")
local translation_control = require("control.translation")

local controller = {}

local Controller = {}
Controller.__index = Controller

function controller.new(control)
    return setmetatable({
        translation = translation_control.new(control),
        attitude = attitude_control.new(control),
        allocation = allocation_control.new(control),
        lastTerms = {},
    }, Controller)
end

function Controller:update(input)
    local state = input.state
    local target = input.target
    local translation = self.translation:update({
        state = state,
        target = target,
        dt = input.dt,
    })
    local attitudeCommands = self.attitude:update({
        state = state,
        commanded = translation.attitude,
        heading = target.heading.angle,
        headingError = target.heading.error,
        dt = input.dt,
    })
    local command = self.allocation:update({
        pose = state.body.pose,
        rawCommands = {
            collective = translation.collective,
            roll = attitudeCommands.roll,
            pitch = attitudeCommands.pitch,
            yaw = attitudeCommands.yaw,
        },
    })
    local translationTerms = self.translation:terms()

    self.lastTerms = {
        horizontal = translationTerms.horizontal,
        vertical = translationTerms.vertical,
        attitude = self.attitude:terms(),
        allocation = self.allocation:terms(),
    }

    return command
end

function Controller:terms()
    return self.lastTerms
end

return controller
