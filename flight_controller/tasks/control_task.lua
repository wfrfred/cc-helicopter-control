local actuator_protocol = require("protocol.actuator")
local config = require("config")
local Controller = require("control.controller")
local flight_state = require("state.flight_state")
local heading_lock = require("state.heading_lock")
local height_lock = require("state.height_lock")
local mixer_module = require("hardware.mixer")
local mode_state = require("state.mode_state")
local rotor_phase = require("hardware.rotor_phase")
local telemetryTerms = require("telemetry.terms")
local trajectory = require("trajectory")
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

local function publishWaiting(shared, flight, state, now)
    shared.telemetryTime = now
    shared.telemetry = telemetryTerms.waiting({
        flight = flight,
        state = state,
        now = now,
    })
end

local function publishRunning(shared, input)
    shared.telemetryTime = input.now
    shared.telemetry = telemetryTerms.running(input)
end

local function makeInitialMachines(initialState)
    return {
        flight = flight_state.new(),
        mode = mode_state.new(initialState, config),
        height = height_lock.new({
            initial_target = initialState.body.pose.height,
            target_rate = config.control.vertical.target_rate,
            rate_deadband = config.control.vertical.lock.speed_deadband,
            relock_timeout = config.control.vertical.lock.relock_timeout,
        }),
        heading = heading_lock.new({
            initial_heading = initialState.navigation.heading.angle,
            lookahead_rate = config.control.heading.lookahead_rate,
            lookahead_time_constant = config.control.heading.lookahead_time_constant,
            rate_deadband = config.control.heading.lock.rate_deadband,
            relock_timeout = config.control.heading.lock.relock_timeout,
        }),
        trajectory = trajectory.new(),
        controller = Controller.new(config.control),
        mixer = mixer_module.new(config.hardware.rotor, config.calibration.rotor),
        phase = rotor_phase.new(config.hardware.rotor),
        actuator = actuator_protocol.new(config.hardware.rotor),
    }
end

function control_task.run(shared)
    local machines = nil
    local lastLoopTime = os.clock() - config.control.loop.dt
    local telemetryTimer = 0.0
    local flightMachine = flight_state.new()

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local input, inputAge, inputStale = readInputOrDefault(shared, loopStart)
        local state = shared.state
        local flight = flightMachine:update({
            state = state,
            input = input,
            inputStale = inputStale,
            now = loopStart,
        })

        if flight.name == "waiting_sensors" then
            publishWaiting(shared, flight, state, loopStart)
            sleep(0.1)
        else
            if machines == nil then
                machines = makeInitialMachines(state)
                flightMachine = machines.flight
            end

            local navigationCommand = takeNavigationCommand(shared)
            local inputEvent = inputEventSnapshot(input)
            local mode = machines.mode:update({
                input = input,
                state = state,
                navigationCommand = navigationCommand,
                dt = dt,
            })
            local navigationExited = mode.transition.navigationExited

            consumeCruiseToggle(shared, input)
            local height = machines.height:update({
                climb = input.manual.velocity.up,
                height = state.body.pose.height,
                verticalSpeed = state.world.velocity.y,
                dt = dt,
            })
            local heading = machines.heading:update({
                headingInput = input.manual.heading.rate,
                heading = state.navigation.heading.angle,
                headingRate = state.navigation.heading.rate,
                dt = dt,
            })

            if navigationExited and input.manual.velocity.up == 0.0 then
                height = machines.height:lockedTarget(state.body.pose.height)
            end

            if navigationExited and input.manual.heading.rate == 0.0 then
                heading = machines.heading:lockedTarget(state.navigation.heading.angle)
            end

            local target = machines.trajectory:update({
                mode = mode,
                input = input,
                state = state,
                height = height,
                heading = heading,
                dt = dt,
            })
            local command = machines.controller:update({
                state = state,
                target = target,
                dt = dt,
            })
            local controlTerms = machines.controller:terms()
            local rotorOutput = machines.mixer:update({
                commands = command,
                phase = machines.phase:read(),
            })

            machines.actuator:send(rotorOutput)

            shared.commands = command
            shared.controlTerms = controlTerms

            telemetryTimer = telemetryTimer + dt
            if telemetryTimer >= config.control.loop.telemetry_dt then
                telemetryTimer = 0.0
                publishRunning(shared, {
                    now = loopStart,
                    dt = dt,
                    input = input,
                    inputEvent = inputEvent,
                    inputAge = inputAge,
                    inputStale = inputStale,
                    inputSender = shared.inputSender,
                    state = state,
                    flight = flight,
                    mode = mode,
                    height = height,
                    heading = heading,
                    target = target,
                    command = command,
                    control = controlTerms,
                    rotor = rotorOutput.blades,
                })
            end

            sleepLoop(loopStart)
        end
    end
end

return control_task
