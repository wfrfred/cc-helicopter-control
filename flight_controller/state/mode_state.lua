local cruise_mode = require("modes.cruise")
local mode_common = require("modes.common")
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

function mode_state.new(initialState, config)
    return setmetatable({
        name = modes.position_hold,
        manual = manual_mode.new(config.control),
        positionHold = position_hold_mode.new(initialState),
        cruise = cruise_mode.new(),
        navigation = navigation_mode.new(config.navigation),
        lastReset = {
            horizontal = false,
        },
        lastTransition = {
            navigationExited = false,
        },
        lastNavigation = nil,
    }, State)
end

local function selectMode(self, manualActive)
    if self.navigation:isActive() then
        return modes.navigation
    end

    if self.cruise:isActive() then
        return modes.cruise
    end

    if manualActive then
        return modes.manual
    end

    return modes.position_hold
end

function State:update(input)
    local command = input.navigationCommand
    local state = input.state
    local dt = input.dt
    local manualInput = input.input
    local resetHorizontal = false
    local navigationWasActive = self.navigation:isActive()

    self.manual:update(manualInput, dt)

    local manualActive = manual_mode.active(manualInput)
    local navigationResult = self.navigation:update(command, state, dt)

    if navigationResult.active then
        self.cruise:clear()
    end

    if self.navigation:cancelForManualInput(manualInput) then
        navigationResult = self.navigation:state()
    end

    if self.cruise:update(manualInput, state, manualActive) then
        resetHorizontal = true
    end

    local selected = selectMode(self, manualActive)
    local navigationExited = navigationWasActive and not self.navigation:isActive()

    if selected ~= self.name then
        self.name = selected
        resetHorizontal = true

        if selected == modes.manual or selected == modes.position_hold then
            self.positionHold:capture(state)
        end
    end

    self.lastReset = {
        horizontal = resetHorizontal,
    }
    self.lastTransition = {
        navigationExited = navigationExited,
    }
    self.lastNavigation = navigationResult

    return {
        name = self.name,
        transition = self.lastTransition,
    }
end

function State:target(input)
    local context = {
        source = self.name,
        input = input.input,
        state = input.state,
        vertical = mode_common.verticalFromLock(input.height),
        heading = mode_common.headingFromLock(input.heading),
        navigation = self.lastNavigation or self.navigation:state(),
        dt = input.dt,
    }

    if self.name == modes.manual then
        return self.manual:target(context)
    end

    if self.name == modes.cruise then
        return self.cruise:target(context)
    end

    if self.name == modes.navigation then
        return self.navigation:target(context)
    end

    return self.positionHold:target(context)
end

function State:terms()
    return {
        mode = {
            name = self.name,
        },
        reset = {
            horizontal = self.lastReset.horizontal,
        },
        transition = {
            navigationExited = self.lastTransition.navigationExited,
        },
        manual = self.manual:snapshot(),
        position_hold = self.positionHold:snapshot(),
        cruise = self.cruise:snapshot(),
        navigation = self.lastNavigation or self.navigation:state(),
    }
end

return mode_state
