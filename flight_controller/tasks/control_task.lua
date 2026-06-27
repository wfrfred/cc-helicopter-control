local actuator_protocol = require("protocol.actuator")
local flight_system = require("app.flight_system")
local config = require("config")
local rotor_phase = require("hardware.rotor_phase")
local telemetryTerms = require("telemetry.terms")
local input_protocol = require("protocol.input")

local control_task = {}

local function clampDt(dt)
    if dt <= 0 then
        return config.control.loop.dt
    end

    return math.min(dt, config.control.loop.max_dt)
end

local function readInputOrDefault(shared, now)
    local inputAge = now - shared.inputTime

    if inputAge > config.control.input.stale_dt then
        return input_protocol.defaultInput(), inputAge, true
    end

    return shared.input, inputAge, false
end

local function takeNavigationCommand(shared)
    local command = shared.navigationCommand
    shared.navigationCommand = nil

    return command
end

local function consumeCruiseToggle(shared, input)
    if shared.input == input then
        shared.input.event.cruiseToggle = false
    end
end

local function inputEventSnapshot(input)
    return {
        cruiseToggle = input.event.cruiseToggle,
        holdCapture = input.event.holdCapture,
    }
end

local function sleepLoop(loopStart)
    local elapsed = os.clock() - loopStart
    local remain = config.control.loop.dt - elapsed

    if remain > 0 then
        sleep(remain)
    else
        sleep(0)
    end
end

local function publishWaiting(shared, state, now)
    shared.telemetryTime = now
    shared.telemetry = telemetryTerms.waiting({
        state = state,
        now = now,
    })
end

local function makeRuntime(initialState)
    return {
        flight = flight_system.new(initialState, config),
        phase = rotor_phase.new(config.hardware.rotor),
        actuator = actuator_protocol.new(config.hardware.rotor),
    }
end

function control_task.run(shared)
    local runtime = nil
    local lastLoopTime = os.clock() - config.control.loop.dt

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local input, inputAge, inputStale = readInputOrDefault(shared, loopStart)
        local state = shared.state

        if not flight_system.ready(state) then
            runtime = nil
            publishWaiting(shared, state, loopStart)
            sleep(0.1)
        else
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
