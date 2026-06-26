local cruise_mode = require("modes.cruise")
local manual_mode = require("modes.manual")
local navigation_mode = require("modes.navigation")
local position_hold_mode = require("modes.position_hold")

--- Coordinates mode lifecycle and dispatches to the active mode.
---
--- Mode interface:
---
--- - `enter(ctx)`
---   Called when the mode becomes active. May update internal mode state.
---
--- - `update(ctx) -> { target, terms }`
---   Advances mode-local state for one control tick and returns this frame's
---   controller target plus mode-local telemetry/debug terms.
---
--- - `exit(ctx)`
---   Called when the mode is deactivated.
---
--- mode_state owns all mode transitions.
--- Modes must not choose or enter another mode directly.
---
--- This module may depend only on the interface above.
--- Do not read or write implementation-specific fields of any mode here.
--- If another view is required, extend the interface deliberately for every mode.

local mode_state = {}

local State = {}
State.__index = State

local modes = {
    manual = "manual",
    position_hold = "position_hold",
    cruise = "cruise",
    navigation = "navigation",
}

function mode_state.new(initialState, config)
    return setmetatable({
        name = modes.position_hold,
        modes = {
            manual = manual_mode.new(initialState, config.control),
            position_hold = position_hold_mode.new(initialState, config.control),
            cruise = cruise_mode.new(),
            navigation = navigation_mode.new(config.navigation),
        },
        lastManualLateral = false,
    }, State)
end

local function manualOverrideActive(input)
    return manual_mode.active(input) or input.manual.velocity.up ~= 0.0
end

function State:update(request)
    local command = request.navigationCommand
    local manualInput = request.input
    local ctx = {
        input = request.input,
        state = request.state,
        dt = request.dt,
    }
    local lateralActive = manual_mode.active(manualInput)
    local overrideActive = manualOverrideActive(manualInput)
    local lateralEdge = lateralActive and not self.lastManualLateral
    local nextMode = self.name

    if self.name == modes.navigation and overrideActive then
        if lateralActive then
            nextMode = modes.manual
        else
            nextMode = modes.position_hold
        end
    elseif lateralEdge then
        nextMode = modes.manual
    elseif manualInput.event.cruiseToggle then
        if self.name == modes.manual then
            nextMode = modes.cruise
        end
    elseif command ~= nil and command.action == "activate" and not overrideActive then
        nextMode = modes.navigation
    elseif command ~= nil
        and command.action == "cancel"
        and self.name == modes.navigation
        and not overrideActive then
        nextMode = modes.position_hold
    elseif self.name == modes.manual and not lateralActive then
        nextMode = modes.position_hold
    end

    if nextMode ~= self.name then
        self.modes[self.name]:exit(ctx)
        self.name = nextMode
        if self.name == modes.navigation then
            self.modes[self.name]:enter({
                input = ctx.input,
                state = ctx.state,
                dt = ctx.dt,
                command = command,
            })
        else
            self.modes[self.name]:enter(ctx)
        end
    elseif self.name == modes.navigation
        and command ~= nil
        and command.action == "activate"
        and not overrideActive then
        self.modes[self.name]:enter({
            input = ctx.input,
            state = ctx.state,
            dt = ctx.dt,
            command = command,
        })
    end

    local result = self.modes[self.name]:update(ctx)
    result.name = self.name

    self.lastManualLateral = lateralActive

    return result
end

return mode_state
