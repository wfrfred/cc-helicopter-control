local Controller = require("control.controller")
local mixer_module = require("hardware.mixer")
local mode_state = require("app.mode_state")
local tablex = require("lib.tablex")
local telemetryTerms = require("telemetry.terms")

local flight_system = {}

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

local function assertAxis(path, value)
    assertFiniteNumber(path .. ".roll", value.roll)
    assertFiniteNumber(path .. ".pitch", value.pitch)
    assertFiniteNumber(path .. ".yaw", value.yaw)
end

local function assertInitialState(state)
    assertFiniteNumber("initialState.body.pose.height", state.body.pose.height)
    assertFiniteNumber("initialState.navigation.heading.angle", state.navigation.heading.angle)
    assertVector("initialState.world.position", state.world.position)
end

local function assertControlState(state)
    assertVector("state.world.position", state.world.position)
    assertVector("state.world.velocity", state.world.velocity)
    assertFiniteTree("state.body.frame", state.body.frame)
    assertFiniteNumber("state.body.pose.height", state.body.pose.height)
    assertFiniteNumber("state.body.pose.roll", state.body.pose.roll)
    assertFiniteNumber("state.body.pose.pitch", state.body.pose.pitch)
    assertFiniteNumber("state.body.pose.heading", state.body.pose.heading)
    assertAxis("state.body.angular.velocity", state.body.angular.velocity)
    assertFiniteNumber("state.navigation.heading.angle", state.navigation.heading.angle)
    assertFiniteNumber("state.navigation.heading.rate", state.navigation.heading.rate)
end

function flight_system.ready(state)
    return state ~= nil
        and state.world ~= nil
        and state.world.position ~= nil
        and state.world.velocity ~= nil
        and state.body ~= nil
        and state.body.frame ~= nil
        and state.body.frame.origin ~= nil
        and state.body.frame.qWorldFromLocal ~= nil
        and state.body.pose ~= nil
        and state.body.pose.height ~= nil
        and state.body.angular ~= nil
        and state.body.angular.velocity ~= nil
        and state.navigation ~= nil
        and state.navigation.heading ~= nil
        and state.navigation.heading.angle ~= nil
        and state.navigation.heading.rate ~= nil
        and state.time ~= nil
        and state.time.pose ~= nil
        and state.time.velocity ~= nil
        and state.time.angularVelocity ~= nil
end

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
