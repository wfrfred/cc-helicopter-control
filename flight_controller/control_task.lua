local Controller = require("controller")
local mathx = require("lib.mathx")
local rotor = require("rotor")
local target_state = require("target_state")
local navigation = require("navigation")
local position_hold = require("position_hold")
local rate_lock = require("rate_lock")
local telemetry_builder = require("telemetry_builder")
local config = require("config")

local control_task = {}

local CONTROL = config.control

local function clampDt(dt)
    if dt <= 0 then
        return CONTROL.loop_dt
    end

    return math.min(dt, CONTROL.max_dt)
end

local ZERO_INPUT = {
    roll = 0.0,
    pitch = 0.0,
    yaw = 0.0,
    climb = 0.0,
}

local function readInput(shared, now)
    local input = shared.input
    local inputAge = now - shared.inputTime

    if inputAge > CONTROL.input_stale_dt then
        return ZERO_INPUT, inputAge, true
    end

    return input, inputAge, false
end

local function stateReady(state)
    return state ~= nil and
        state.body.pose ~= nil and
        state.body.rates ~= nil and
        state.body.velocity ~= nil
end

local function waitForSensors(shared)
    while shared.running and not stateReady(shared.state) do
        local state = shared.state
        local haveState = state ~= nil
        local now = os.clock()

        shared.telemetryTime = now
        shared.telemetry = {
            status = "waiting_sensors",
            time = now,
            havePose = haveState and state.body.pose ~= nil,
            haveRates = haveState and state.body.rates ~= nil,
            haveVelocity = haveState and state.body.velocity ~= nil,
        }

        sleep(0.1)
    end
end

function control_task.run(shared)
    local mixer = rotor.new(config.hardware.rotor, config.calibration.rotor, config.calibration.mixer_axis)

    waitForSensors(shared)

    local initialState = shared.state
    local initial = initialState.body.pose

    local targets = target_state.new(initial, CONTROL)
    local positionHold = position_hold.new(CONTROL)
    local positionTarget = navigation.makePositionTarget(initialState)
    local positionHoldActive = false
    local downLock = rate_lock.new({
        initial_target = initial.down,
        target_rate = CONTROL.height_target_rate,
        rate_deadband = CONTROL.height_lock_speed_deadband,
        relock_timeout = CONTROL.height_lock_relock_timeout,
    })
    local yawLock = rate_lock.new({
        initial_target = initial.yaw,
        target_rate = CONTROL.yaw_target_rate,
        rate_deadband = CONTROL.yaw_lock_rate_deadband,
        relock_timeout = CONTROL.yaw_lock_relock_timeout,
        error = function(target, current)
            return mathx.wrapPi(target - current)
        end,
    })
    local controller = Controller.new(CONTROL)

    local lastLoopTime = os.clock() - CONTROL.loop_dt
    local telemetryTimer = 0.0

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local input, inputAge, inputStale = readInput(shared, loopStart)
        targets:update(input, dt)

        local now = os.clock()
        local state = shared.state
        local pose = state.body.pose
        local rates = state.body.rates
        local velocity = state.body.velocity
        local poseAge = now - state.time.pose
        local ratesAge = now - state.time.rates
        local velocityAge = now - state.time.velocity

        local positionResult
        local positionManual = input.roll ~= 0 or input.pitch ~= 0
        if positionManual then
            positionTarget = navigation.makePositionTarget(state)
            if positionHoldActive then
                positionHold:reset()
            end
            positionHoldActive = false
            positionResult = position_hold.inactive()
        else
            if not positionHoldActive then
                positionTarget = navigation.makePositionTarget(state)
                positionHoldActive = true
            end

            positionResult = positionHold:update(
                navigation.projectPositionTargetErrorToBodyFrd(positionTarget, state),
                velocity,
                dt
            )
            targets.roll = positionResult.roll
            targets.pitch = positionResult.pitch
        end

        local heightResult = downLock:update(-input.climb, pose.down, velocity.down, dt)
        local yawResult = yawLock:update(input.yaw, pose.yaw, rates.yaw, dt)

        local result = controller:update({
            targets = targets,
            pose = pose,
            rollRate = rates.roll,
            pitchRate = rates.pitch,
            yawRate = rates.yaw,
            velocity = velocity,
            height = heightResult,
            yaw = yawResult,
            dt = dt,
        })

        mixer:set(
            result.commands.collective,
            result.commands.roll,
            result.commands.yaw,
            result.commands.pitch
        )
        local rotorOutput = mixer:update()

        telemetryTimer = telemetryTimer + dt
        if telemetryTimer >= CONTROL.telemetry_dt then
            telemetryTimer = 0.0

            shared.telemetryTime = now
            shared.telemetry = telemetry_builder.running({
                shared = shared,
                state = state,
                input = input,
                rotorOutput = rotorOutput,
                controllers = controller:pidControllers(),
                positionControllers = positionHold:pidControllers(),
                commands = result.commands,
                terms = result.terms,
                position = positionResult,

                time = now,
                dt = dt,
                poseAge = poseAge,
                ratesAge = ratesAge,
                velocityAge = velocityAge,
                inputAge = inputAge,
                inputStale = inputStale,
            })
        end

        local elapsed = os.clock() - loopStart
        local remain = CONTROL.loop_dt - elapsed

        if remain > 0 then
            sleep(remain)
        else
            sleep(0)
        end
    end
end

return control_task
