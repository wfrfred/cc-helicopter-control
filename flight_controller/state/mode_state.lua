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

local modeOrder = {
    modes.manual,
    modes.position_hold,
    modes.cruise,
    modes.navigation,
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
        lastNavigationResult = nil,
        lastManualLateral = false,
        lastState = initialState,
    }, State)
end

local function activeMode(self)
    local mode = self.modes[self.name]

    assert(mode ~= nil, "unknown mode: " .. tostring(self.name))

    return mode
end

local function manualLateralActive(input)
    return manual_mode.active(input)
end

local function manualOverrideActive(input)
    return manualLateralActive(input) or input.manual.velocity.up ~= 0.0
end

local function fixedControlTerms(state, source)
    return {
        height = {
            height = state.body.pose.height,
            speed = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = source,
        },
        heading = {
            angle = state.navigation.heading.angle,
            rate = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = source,
        },
        lock = {
            height = source,
            heading = source,
        },
    }
end

local function requestContext(input)
    return {
        input = input.input,
        state = input.state,
        dt = input.dt,
        command = input.navigationCommand,
        reason = input.reason,
        current = nil,
        navigationExited = false,
    }
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

    for _, other in ipairs(exclusive[name] or {}) do
        if other ~= name then
            self.modes[other]:exit(exitCtx)
        end
    end

    self.name = name

    local status = self.modes[name]:enter(exitCtx) or {
        active = true,
    }

    self.lastReset.horizontal = true

    return status
end

function State:update(input)
    local command = input.navigationCommand
    local manualInput = input.input
    local ctx = requestContext(input)
    local wasNavigation = self.name == modes.navigation
    local lateralActive = manualLateralActive(manualInput)
    local overrideActive = manualOverrideActive(manualInput)
    local lateralEdge = lateralActive and not self.lastManualLateral
    local status = nil
    local statuses = {}

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
            status = enter(self, modes.manual, ctx)
        else
            ctx.reason = modes.position_hold
            status = enter(self, modes.position_hold, ctx)
        end
    elseif lateralEdge then
        ctx.reason = modes.manual
        status = enter(self, modes.manual, ctx)
    elseif manualInput.event.cruiseToggle then
        if self.name == modes.manual then
            ctx.reason = modes.cruise
            status = enter(self, modes.cruise, ctx)
        end
    elseif command ~= nil then
        if not overrideActive then
            ctx.reason = modes.navigation
            status = enter(self, modes.navigation, ctx)
        end
    end

    ctx.current = self.name

    for _, name in ipairs(modeOrder) do
        statuses[name] = self.modes[name]:update(ctx)
    end

    status = statuses[self.name] or status or {
        active = true,
    }

    if not status.active and self.name ~= modes.position_hold then
        ctx.reason = modes.position_hold
        status = enter(self, modes.position_hold, ctx)
        ctx.current = self.name
        ctx.navigationExited = wasNavigation
        status = self.modes[self.name]:update(ctx) or status
    end

    self.lastTransition.navigationExited = wasNavigation and self.name ~= modes.navigation
    self.lastNavigationResult = statuses[modes.navigation].navigation
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
        navigation = self.lastNavigationResult or {
            active = false,
        },
        dt = input.dt,
    }

    return activeMode(self):target(context)
end

function State:terms()
    local mode = activeMode(self)
    local activeTerms = mode:terms(self.lastState) or {}
    local targetControl = activeTerms.control
        or fixedControlTerms(self.lastState, self.name)

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
