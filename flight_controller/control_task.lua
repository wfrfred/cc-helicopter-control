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

local function clampDt(dt)
    if dt <= 0 then
        return config.control.loop_dt
    end

    return math.min(dt, config.control.max_dt)
end

local zeroInput = {
    roll = 0.0,
    pitch = 0.0,
    yaw = 0.0,
    climb = 0.0,
}

local function readInput(shared, now)
    local input = shared.input
    local inputAge = now - shared.inputTime

    if inputAge > config.control.input_stale_dt then
        return zeroInput, inputAge, true
    end

    return input, inputAge, false
end

local function stateReady(state)
    return state ~= nil and
        state.raw.position ~= nil and
        state.raw.velocity ~= nil and
        state.body.pose ~= nil and
        state.body.rates ~= nil and
        state.body.velocity ~= nil and
        state.time.pose ~= nil and
        state.time.rates ~= nil and
        state.time.velocity ~= nil
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

local function verticalMode(lockResult)
    if lockResult.active then
        return "height_hold"
    end

    if lockResult.pending then
        return "height_hold_pending"
    end

    return "manual_climb"
end

local function yawMode(lockResult)
    if lockResult.active then
        return "yaw_hold"
    end

    if lockResult.pending then
        return "yaw_hold_pending"
    end

    return "manual_yaw"
end

function control_task.run(shared)
    local mixer = rotor.new(config.hardware.rotor, config.calibration.rotor, config.calibration.mixer_axis)

    waitForSensors(shared)

    local initialState = shared.state
    local initial = initialState.body.pose

    local manualAttitude = target_state.new(initial, config.control)
    local positionHold = position_hold.new(config.control)
    local positionTarget = navigation.makePositionTarget(initialState)
    local positionHoldActive = false
    local downLock = rate_lock.new({
        initial_target = initial.down,
        target_rate = config.control.height_target_rate,
        rate_deadband = config.control.height_lock_speed_deadband,
        relock_timeout = config.control.height_lock_relock_timeout,
    })
    local yawLock = rate_lock.new({
        initial_target = initial.yaw,
        target_rate = config.control.yaw_target_rate,
        rate_deadband = config.control.yaw_lock_rate_deadband,
        relock_timeout = config.control.yaw_lock_relock_timeout,
        error = function(target, current)
            return mathx.wrapPi(target - current)
        end,
    })
    local controller = Controller.new(config.control)
    local flight = {
        mode = {},
        target = {},
    }

    local lastLoopTime = os.clock() - config.control.loop_dt
    local telemetryTimer = 0.0

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local input, inputAge, inputStale = readInput(shared, loopStart)
        manualAttitude:update(input, dt)

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
        local attitudeTarget

        if positionManual then
            positionTarget = navigation.makePositionTarget(state)
            if positionHoldActive then
                positionHold:reset()
            end
            positionHoldActive = false
            positionResult = position_hold.inactive()
            flight.mode.lateral = "manual_attitude"
            flight.target.position = nil
            attitudeTarget = {
                roll = manualAttitude.roll,
                pitch = manualAttitude.pitch,
                source = flight.mode.lateral,
            }
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
            flight.mode.lateral = "position_hold"
            flight.target.position = positionTarget
            attitudeTarget = {
                roll = positionResult.roll,
                pitch = positionResult.pitch,
                source = flight.mode.lateral,
            }
        end

        local verticalLock = downLock:update(-input.climb, pose.down, velocity.down, dt)
        local yawLockResult = yawLock:update(input.yaw, pose.yaw, rates.yaw, dt)

        flight.mode.vertical = verticalMode(verticalLock)
        flight.mode.yaw = yawMode(yawLockResult)
        flight.target.attitude = attitudeTarget
        flight.target.vertical = {
            down = verticalLock.target,
            rate = verticalLock.commandedRate,
            active = verticalLock.active,
            pending = verticalLock.pending,
            error = verticalLock.error,
            source = flight.mode.vertical,
        }
        flight.target.yaw = {
            angle = yawLockResult.target,
            rate = yawLockResult.commandedRate,
            active = yawLockResult.active,
            pending = yawLockResult.pending,
            error = yawLockResult.error,
            source = flight.mode.yaw,
        }

        local result = controller:update({
            target = flight.target,
            state = state.body,
            dt = dt,
        })

        mixer:setCommands(result.commands)
        local rotorOutput = mixer:update()

        shared.target = flight.target
        shared.controlResult = result
        shared.commands = result.commands

        telemetryTimer = telemetryTimer + dt
        if telemetryTimer >= config.control.telemetry_dt then
            telemetryTimer = 0.0

            shared.telemetryTime = now
            shared.telemetry = telemetry_builder.running({
                shared = shared,
                state = state,
                input = input,
                flight = flight,
                result = result,
                rotorOutput = rotorOutput,
                controllers = controller:pidControllers(),
                positionControllers = positionHold:pidControllers(),
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
        local remain = config.control.loop_dt - elapsed

        if remain > 0 then
            sleep(remain)
        else
            sleep(0)
        end
    end
end

return control_task
