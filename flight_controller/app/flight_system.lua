local Controller = require("control.controller")
local mixer_module = require("hardware.mixer")
local mode_state = require("app.mode_state")
local tablex = require("lib.tablex")
local telemetryTerms = require("telemetry.terms")

local flight_system = {}

local System = {}
System.__index = System

function flight_system.ready(state)
    return state ~= nil
        and state.world ~= nil
        and state.world.position ~= nil
        and state.world.velocity ~= nil
        and state.body ~= nil
        and state.body.frame ~= nil
        and state.body.frame.forward ~= nil
        and state.body.frame.right ~= nil
        and state.body.frame.down ~= nil
        and state.body.orientation ~= nil
        and state.body.pose ~= nil
        and state.body.pose.height ~= nil
        and state.body.angular ~= nil
        and state.body.angular.velocity ~= nil
        and state.navigation ~= nil
        and state.navigation.heading ~= nil
        and state.navigation.heading.angle ~= nil
        and state.navigation.heading.rate ~= nil
        and state.navigation.velocity ~= nil
        and state.time ~= nil
        and state.time.pose ~= nil
        and state.time.velocity ~= nil
        and state.time.angularVelocity ~= nil
end

function flight_system.new(initialState, config)
    return setmetatable({
        mode = mode_state.new(initialState, config),
        controller = Controller.new(config.control),
        mixer = mixer_module.new(config.hardware.rotor, config.calibration.rotor),
        telemetryDt = config.control.loop.telemetry_dt,
        telemetryTimer = 0.0,
    }, System)
end

function System:update(frame)
    local modeResult = self.mode:update({
        input = frame.input,
        state = frame.state,
        navigationCommand = frame.navigationCommand,
        dt = frame.dt,
    })
    local controlResult = self.controller:update({
        state = frame.state,
        target = modeResult.target,
        dt = frame.dt,
    })
    local rotorResult = self.mixer:update({
        commands = controlResult.output,
        phase = frame.rotorPhase,
    })

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
        command = controlResult.output,
        controlTerms = controlResult.terms,
        rotor = rotorResult,
        telemetry = telemetry,
    }
end

return flight_system
