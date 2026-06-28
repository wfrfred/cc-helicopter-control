local actuator_protocol = require("protocol.actuator")
local flight_system = require("app.flight_system")
local config = require("config")
local rotor_phase = require("hardware.rotor_phase")
local telemetryTerms = require("telemetry.terms")
local input_protocol = require("protocol.input")
local control_state = require("app.control_state")

local control_task = {}

---@param dt number
---@return number
local function clampDt(dt)
    if dt <= 0 then
        return config.control.loop.dt
    end

    return math.min(dt, config.control.loop.max_dt)
end

---@param shared table
---@param now number
---@return table, number, boolean
local function readInputOrDefault(shared, now)
    local inputAge = now - shared.inputTime

    if inputAge > config.control.input.stale_dt then
        return input_protocol.defaultInput(), inputAge, true
    end

    return shared.input, inputAge, false
end

---@param shared table
---@return table|nil
local function takeNavigationCommand(shared)
    local command = shared.navigationCommand
    shared.navigationCommand = nil

    return command
end

---@param shared table
---@param input table
local function consumeCruiseToggle(shared, input)
    if shared.input == input then
        shared.input.event.cruiseToggle = false
    end
end

---@param input table
---@return table
local function inputEventSnapshot(input)
    return {
        cruiseToggle = input.event.cruiseToggle,
        holdCapture = input.event.holdCapture,
    }
end

---@param loopStart number
local function sleepLoop(loopStart)
    local elapsed = os.clock() - loopStart
    local remain = config.control.loop.dt - elapsed

    if remain > 0 then
        sleep(remain)
    else
        sleep(0)
    end
end

---@param shared table
---@param state ControlState|nil
---@param now number
local function publishWaiting(shared, state, now)
    shared.telemetryTime = now
    shared.telemetry = telemetryTerms.waiting({
        state = state,
        now = now,
    })
end

---@param initialState ControlState
---@return { flight: FlightSystem, phase: table, actuator: table }
local function makeRuntime(initialState)
    return {
        flight = flight_system.new(initialState, config),
        phase = rotor_phase.new(config.hardware.rotor),
        actuator = actuator_protocol.new(config.hardware.rotor),
    }
end

---@param shared table
---@return ControlState|nil
local function readControlState(shared)
    local samples = shared.sensors

    if not control_state.ready(samples) then
        return nil
    end

    return control_state.fromSensors(samples, {
        bodyAxis = config.calibration.body_axis,
    })
end

---@param shared table
function control_task.run(shared)
    local runtime = nil
    local lastLoopTime = os.clock() - config.control.loop.dt

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local input, inputAge, inputStale = readInputOrDefault(shared, loopStart)
        local state = readControlState(shared)

        if not flight_system.ready(state) then
            runtime = nil
            publishWaiting(shared, state, loopStart)
            sleep(0.1)
        else
            ---@cast state ControlState
            if runtime == nil then
                runtime = makeRuntime(state)
            end

            local navigationCommand = takeNavigationCommand(shared)
            local inputEvent = inputEventSnapshot(input)
            local flightResult = runtime.flight:update({
                now = loopStart,
                dt = dt,
                input = input,
                inputEvent = inputEvent,
                inputAge = inputAge,
                inputStale = inputStale,
                inputSender = shared.inputSender,
                state = state,
                navigationCommand = navigationCommand,
                navigationConfig = config.navigation,
                rotorPhase = runtime.phase:read(),
            })

            consumeCruiseToggle(shared, input)

            runtime.actuator:send(flightResult.rotorResult)

            shared.commands = flightResult.controlResult.output
            shared.controlTerms = flightResult.controlResult.terms

            if flightResult.telemetry ~= nil then
                shared.telemetryTime = loopStart
                shared.telemetry = flightResult.telemetry
            end

            sleepLoop(loopStart)
        end
    end
end

return control_task
