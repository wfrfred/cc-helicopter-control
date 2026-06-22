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

function mode_state.new(initialState, config)
    return setmetatable({
        name = modes.position_hold,
        manual = manual_mode.new(config.control),
        positionHold = position_hold_mode.new(initialState),
        cruise = cruise_mode.new(),
        navigation = navigation_mode.new(config.navigation),
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

    return {
        name = self.name,
        manualAttitude = self.manual:snapshot(),
        positionTarget = self.positionHold:snapshot(),
        cruiseVelocity = self.cruise:snapshot(),
        navigation = navigationResult,
        reset = {
            horizontal = resetHorizontal,
        },
        transition = {
            navigationExited = navigationExited,
        },
    }
end

return mode_state
