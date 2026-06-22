local navigation = require("navigation")
local mathx = require("lib.mathx")

local mode_state = {}

local State = {}
State.__index = State

local modes = {
    manual = "manual",
    position_hold = "position_hold",
    cruise = "cruise",
    navigation = "navigation",
}

local function horizontalInput(input)
    return input.manual.attitude.roll ~= 0.0
        or input.manual.attitude.pitch ~= 0.0
        or input.manual.heading.rate ~= 0.0
end

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

local function motion(state)
    return {
        worldVelocity = horizontalVector(state.world.velocity),
        verticalSpeed = state.world.velocity.y,
        headingRate = state.navigation.heading.rate,
    }
end

local function moveToward(x, target, rate, dt)
    local d = target - x
    local step = rate * dt

    if math.abs(d) <= step then
        return target
    end

    if d > 0 then
        return x + step
    end

    return x - step
end

function mode_state.new(initialState, config)
    return setmetatable({
        control = config.control,
        navigator = navigation.new(config.navigation),
        name = modes.position_hold,
        positionTarget = horizontalVector(initialState.world.position),
        cruiseVelocity = nil,
        cruiseManualReleasePending = false,
        cruiseToggleHeld = false,
        manualRoll = config.control.attitude.home.roll,
        manualPitch = config.control.attitude.home.pitch,
    }, State)
end

function State:updateManualAttitude(input, dt)
    local control = self.control
    local roll = input.manual.attitude.roll
    local pitch = input.manual.attitude.pitch

    if roll ~= 0.0 then
        self.manualRoll = mathx.clamp(
            self.manualRoll + roll * control.attitude.target_rate.roll * dt,
            -control.attitude.limit.roll,
            control.attitude.limit.roll
        )
    else
        self.manualRoll = moveToward(
            self.manualRoll,
            control.attitude.home.roll,
            control.attitude.center_rate.roll,
            dt
        )
    end

    if pitch ~= 0.0 then
        self.manualPitch = mathx.clamp(
            self.manualPitch + pitch * control.attitude.target_rate.pitch * dt,
            -control.attitude.limit.pitch,
            control.attitude.limit.pitch
        )
    else
        self.manualPitch = moveToward(
            self.manualPitch,
            control.attitude.home.pitch,
            control.attitude.center_rate.pitch,
            dt
        )
    end
end

local function selectMode(self, input)
    if self.navigator:isActive() then
        return modes.navigation
    end

    if self.cruiseVelocity ~= nil then
        return modes.cruise
    end

    if horizontalInput(input) then
        return modes.manual
    end

    return modes.position_hold
end

local function updateNavigationCommand(self, command, state, dt)
    if command == nil or command.action == nil then
        if self.navigator:isActive() then
            return self.navigator:update(state, dt, motion(state))
        end

        return self.navigator:state()
    end

    local result = self.navigator:command(command, state, motion(state))

    if result.active then
        self.cruiseVelocity = nil

        if result.target == nil then
            result = self.navigator:update(state, dt, motion(state))
        end
    end

    return result
end

local function cancelNavigationForManualInput(self, input)
    if not self.navigator:isActive() then
        return false
    end

    if horizontalInput(input)
        or input.manual.velocity.up ~= 0.0
        or input.manual.heading.rate ~= 0.0 then
        self.navigator:cancel("manual")
        return true
    end

    return false
end

local function updateCruise(self, input, state)
    local manual = horizontalInput(input)
    local cruiseToggle = input.event.cruiseToggle and not self.cruiseToggleHeld

    self.cruiseToggleHeld = input.event.cruiseToggle == true

    if cruiseToggle then
        self.cruiseVelocity = horizontalVector(state.world.velocity)
        self.cruiseManualReleasePending = manual
        return true
    end

    if self.cruiseVelocity == nil then
        return false
    end

    if self.cruiseManualReleasePending then
        if not manual then
            self.cruiseManualReleasePending = false
        end
        return false
    end

    if manual then
        self.cruiseVelocity = nil
        return true
    end

    return false
end

function State:update(input)
    local state = input.state
    local command = input.navigationCommand
    local dt = input.dt
    local resetHorizontal = false
    local navigationWasActive = self.navigator:isActive()

    self:updateManualAttitude(input.input, dt)

    local navigationResult = updateNavigationCommand(self, command, state, dt)

    if cancelNavigationForManualInput(self, input.input) then
        navigationResult = self.navigator:state()
    end

    if updateCruise(self, input.input, state) then
        resetHorizontal = true
    end

    local selected = selectMode(self, input.input)
    local navigationExited = navigationWasActive and not self.navigator:isActive()

    if selected ~= self.name then
        self.name = selected
        resetHorizontal = true

        if selected == modes.manual or selected == modes.position_hold then
            self.positionTarget = horizontalVector(state.world.position)
        end
    end

    return {
        name = self.name,
        manualAttitude = {
            roll = self.manualRoll,
            pitch = self.manualPitch,
        },
        positionTarget = self.positionTarget,
        cruiseVelocity = self.cruiseVelocity,
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
