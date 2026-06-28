local Controller = require("control.controller")
local mixer_module = require("hardware.mixer")
local mode_state = require("app.mode_state")
local tablex = require("lib.tablex")
local telemetryTerms = require("telemetry.terms")

local flight_system = {}

---@class FlightSystemFrame
---@field now number
---@field dt number
---@field input table
---@field inputEvent table
---@field inputAge number
---@field inputStale boolean
---@field inputSender string|nil
---@field state ControlState
---@field navigationCommand table|nil
---@field navigationConfig table
---@field rotorPhase table

---@class FlightSystemResult
---@field controlResult ControlControllerResult
---@field rotorResult table
---@field telemetry table|nil

---@class FlightSystem
---@field mode FlightModeState
---@field controller ControlController
---@field mixer table
---@field telemetryDt number
---@field telemetryTimer number
local System = {}
System.__index = System

local function assertFiniteNumber(path, value)
    assert(type(value) == "number", path .. " must be number")
    assert(value == value, path .. " must not be NaN")
    assert(value ~= math.huge and value ~= -math.huge, path .. " must be finite")
end

local function assertFiniteTree(path, value)
    if type(value) == "number" then
        assertFiniteNumber(path, value)
    elseif type(value) == "table" then
        for key, child in pairs(value) do
            assertFiniteTree(path .. "." .. tostring(key), child)
        end
    end
end

local function assertVector(path, value)
    assertFiniteNumber(path .. ".x", value.x)
    assertFiniteNumber(path .. ".y", value.y)
    assertFiniteNumber(path .. ".z", value.z)
end

local function assertInitialState(state)
    assertFiniteNumber("initialState.navigation.position.z", state.navigation.position.z)
    assertVector("initialState.world.position", state.world.position)
end

local function assertControlState(state)
    assertFiniteTree("state.frames.world", state.frames.world)
    assertFiniteTree("state.frames.navigation", state.frames.navigation)
    assertFiniteTree("state.frames.body", state.frames.body)
    assertVector("state.world.position", state.world.position)
    assertVector("state.world.velocity", state.world.velocity)
    assertFiniteTree("state.world.orientation", state.world.orientation)
    assertVector("state.world.angularVelocity", state.world.angularVelocity)
    assertVector("state.navigation.position", state.navigation.position)
    assertFiniteTree("state.navigation.orientation", state.navigation.orientation)
    assertVector("state.navigation.velocity", state.navigation.velocity)
    assertVector("state.navigation.angularVelocity", state.navigation.angularVelocity)
    assertVector("state.body.position", state.body.position)
    assertFiniteTree("state.body.orientation", state.body.orientation)
    assertVector("state.body.velocity", state.body.velocity)
    assertVector("state.body.angularVelocity", state.body.angularVelocity)
    assertFiniteNumber("state.sampleTime.pose", state.sampleTime.pose)
    assertFiniteNumber("state.sampleTime.velocity", state.sampleTime.velocity)
    assertFiniteNumber("state.sampleTime.angularVelocity", state.sampleTime.angularVelocity)
end

---@param state ControlState|nil
---@return boolean
function flight_system.ready(state)
    return state ~= nil
        and state.frames ~= nil
        and state.frames.world ~= nil
        and state.frames.navigation ~= nil
        and state.frames.body ~= nil
        and state.world ~= nil
        and state.world.position ~= nil
        and state.world.orientation ~= nil
        and state.world.velocity ~= nil
        and state.world.angularVelocity ~= nil
        and state.body ~= nil
        and state.body.position ~= nil
        and state.body.orientation ~= nil
        and state.body.velocity ~= nil
        and state.body.angularVelocity ~= nil
        and state.navigation ~= nil
        and state.navigation.position ~= nil
        and state.navigation.orientation ~= nil
        and state.navigation.velocity ~= nil
        and state.navigation.angularVelocity ~= nil
        and state.sampleTime ~= nil
        and state.sampleTime.pose ~= nil
        and state.sampleTime.velocity ~= nil
        and state.sampleTime.angularVelocity ~= nil
end

---@param initialState ControlState
---@param config table
---@return FlightSystem
function flight_system.new(initialState, config)
    assertInitialState(initialState)

    return setmetatable({
        mode = mode_state.new(initialState, config),
        controller = Controller.new(config.control),
        mixer = mixer_module.new(config.hardware.rotor, config.calibration.rotor),
        telemetryDt = config.control.loop.telemetry_dt,
        telemetryTimer = 0.0,
    }, System)
end

---@param frame FlightSystemFrame
---@return FlightSystemResult
function System:update(frame)
    assertControlState(frame.state)
    assertFiniteTree("rotorPhase", frame.rotorPhase)

    local modeResult = self.mode:update({
        input = frame.input,
        state = frame.state,
        navigationCommand = frame.navigationCommand,
        dt = frame.dt,
    })
    assertFiniteTree("modeResult.target", modeResult.target)

    local controlResult = self.controller:update({
        state = frame.state,
        target = modeResult.target,
        dt = frame.dt,
    })
    assertFiniteTree("controlResult.output", controlResult.output)

    local rotorResult = self.mixer:update({
        commands = controlResult.output,
        phase = frame.rotorPhase,
    })
    assertFiniteTree("rotorResult.blades", rotorResult.blades)

    self.telemetryTimer = self.telemetryTimer + frame.dt

    local telemetry = nil

    if self.telemetryTimer >= self.telemetryDt then
        self.telemetryTimer = 0.0
        telemetry = telemetryTerms.running(tablex.record.merge(frame, {
            flight = {
                name = "running",
                reason = frame.inputStale and "input_stale_zeroed" or "ready",
            },
            modeResult = modeResult,
            controlResult = controlResult,
            rotorResult = rotorResult,
        }))
    end

    return {
        controlResult = controlResult,
        rotorResult = rotorResult,
        telemetry = telemetry,
    }
end

return flight_system
