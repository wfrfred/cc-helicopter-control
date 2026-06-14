local Controller = require("controller")
local mathx = require("lib.mathx")
local rotor = require("rotor")
local target_state = require("target_state")
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

local function waitForSensors(shared)
    while shared.running and (
        shared.poseSnapshot == nil or
        shared.ratesSnapshot == nil or
        shared.velocitySnapshot == nil
    ) do
        local now = os.clock()
        shared.telemetryTime = now
        shared.telemetry = {
            status = "waiting_sensors",
            time = now,
            havePose = shared.poseSnapshot ~= nil,
            haveRates = shared.ratesSnapshot ~= nil,
            haveVelocity = shared.velocitySnapshot ~= nil,
        }

        sleep(0.1)
    end
end

function control_task.run(shared)
    local mixer = rotor.new(config.hardware.rotor, config.calibration.rotor, config.calibration.mixer_axis)

    waitForSensors(shared)

    local initial = shared.poseSnapshot.body.pose

    local targets = target_state.new(initial, CONTROL)
    local positionHold = position_hold.new(initial, CONTROL)
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
        local poseSnapshot = shared.poseSnapshot
        local ratesSnapshot = shared.ratesSnapshot
        local velocitySnapshot = shared.velocitySnapshot
        local pose = poseSnapshot.body.pose
        local rates = ratesSnapshot.body.rates
        local velocity = velocitySnapshot.body.velocity
        local poseAge = now - poseSnapshot.time
        local ratesAge = now - ratesSnapshot.time
        local velocityAge = now - velocitySnapshot.time

        local positionResult = positionHold:update(input, pose, velocity, dt)
        if positionResult.active then
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
                poseSnapshot = poseSnapshot,
                input = input,
                velocitySnapshot = velocitySnapshot,
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
