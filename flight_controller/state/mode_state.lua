local cruise_mode = require("modes.cruise")
local manual_mode = require("modes.manual")
local navigation_mode = require("modes.navigation")
local position_hold_mode = require("modes.position_hold")

local mode_state = {}

local State = {}
State.__index = State

local modes = {
    manual = "manual",
    position_hold = "position_hold",
    cruise = "cruise",
    navigation = "navigation",
}

local exclusive = {
    manual = { modes.navigation, modes.cruise },
    position_hold = { modes.navigation, modes.cruise },
    cruise = { modes.navigation },
    navigation = { modes.cruise },
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
        lastReset = {
            horizontal = false,
        },
        lastTransition = {
            navigationExited = false,
        },
        lastManualLateral = false,
        lastState = initialState,
    }, State)
end

local function manualOverrideActive(input)
    return manual_mode.active(input) or input.manual.velocity.up ~= 0.0
end

local function enter(self, name, ctx)
    local exitCtx = {
        input = ctx.input,
        state = ctx.state,
        dt = ctx.dt,
        command = ctx.command,
        reason = ctx.reason or name,
        from = self.name,
    }

    for _, other in ipairs(exclusive[name]) do
        self.modes[other]:exit(exitCtx)
    end

    self.name = name
    self.modes[name]:enter(exitCtx)
    self.lastReset.horizontal = true
end

function State:update(input)
    local command = input.navigationCommand
    local manualInput = input.input
    local ctx = {
        input = input.input,
        state = input.state,
        dt = input.dt,
        command = input.navigationCommand,
        reason = input.reason,
    }
    local wasNavigation = self.name == modes.navigation
    local lateralActive = manual_mode.active(manualInput)
    local overrideActive = manualOverrideActive(manualInput)
    local lateralEdge = lateralActive and not self.lastManualLateral

    self.lastReset = {
        horizontal = false,
    }
    self.lastTransition = {
        navigationExited = false,
    }
    self.lastState = input.state

    if self.name == modes.navigation and overrideActive then
        if lateralActive then
            ctx.reason = modes.manual
            enter(self, modes.manual, ctx)
        else
            ctx.reason = modes.position_hold
            enter(self, modes.position_hold, ctx)
        end
    elseif lateralEdge then
        ctx.reason = modes.manual
        enter(self, modes.manual, ctx)
    elseif manualInput.event.cruiseToggle then
        if self.name == modes.manual then
            ctx.reason = modes.cruise
            enter(self, modes.cruise, ctx)
        end
    elseif command ~= nil then
        if not overrideActive then
            ctx.reason = modes.navigation
            enter(self, modes.navigation, ctx)
        end
    end

    local status = self.modes[self.name]:update(ctx)

    if not status.active and self.name ~= modes.position_hold then
        ctx.reason = modes.position_hold
        enter(self, modes.position_hold, ctx)
        status = self.modes[self.name]:update(ctx)
    end

    self.lastTransition.navigationExited = wasNavigation and self.name ~= modes.navigation
    self.lastManualLateral = lateralActive

    return {
        name = self.name,
        reset = {
            horizontal = self.lastReset.horizontal,
        },
        transition = self.lastTransition,
    }
end

function State:target(input)
    local context = {
        source = self.name,
        input = input.input,
        state = input.state,
        dt = input.dt,
    }

    return self.modes[self.name]:target(context)
end

function State:terms()
    local mode = self.modes[self.name]
    local activeTerms = mode:terms(self.lastState)
    local targetControl = activeTerms.control

    activeTerms.control = nil

    return {
        mode = {
            name = self.name,
            terms = activeTerms,
        },
        transition = {
            navigationExited = self.lastTransition.navigationExited,
        },
        navigation = self.name == modes.navigation and activeTerms or nil,
        height = targetControl.height,
        heading = targetControl.heading,
        lock = targetControl.lock,
    }
end

return mode_state
