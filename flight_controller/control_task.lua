local Controller = require("controller")
local rotor = require("rotor")
local target_state = require("target_state")
local yaw_lock = require("yaw_lock")
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
        shared.state == nil or
        shared.yawRateTime <= 0.0 or
        shared.velocity == nil or
        shared.velocityTime <= 0.0
    ) do
        local now = os.clock()
        shared.telemetryTime = now
        shared.telemetry = {
            status = "waiting_sensors",
            time = now,
            haveState = shared.state ~= nil,
            haveYawRate = shared.yawRateTime > 0.0,
            haveVelocity = shared.velocity ~= nil,
        }

        sleep(0.1)
    end
end

function control_task.run(shared)
    local mixer = rotor.new(config.hardware.rotor, config.calibration.rotor, config.calibration.mixer_axis)

    waitForSensors(shared)

    local initial = shared.state

    local targets = target_state.new(initial, CONTROL)
    local yawLock = yaw_lock.new(initial.yaw, CONTROL)
    local controller = Controller.new(CONTROL)

    local lastLoopTime = os.clock() - CONTROL.loop_dt
    local telemetryTimer = 0.0

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local input, inputAge, inputStale = readInput(shared, loopStart)
        targets:update(input, dt)

        local state = shared.state
        local stateNow = os.clock()
        local stateTime = shared.stateTime
        local stateAge = stateNow - stateTime
        local yawRate = shared.yawRate
        local velocity = shared.velocity
        local yawRateAge = stateNow - shared.yawRateTime
        local velocityAge = stateNow - shared.velocityTime

        local yawResult = yawLock:update(input.yaw, state.yaw, yawRate)

        local result = controller:update({
            targets = targets,
            state = state,
            yawRate = yawRate,
            velocity = velocity,
            yaw = yawResult,
            dt = dt,
        })

        local commands = result.commands
        local terms = result.terms

        mixer:set(commands.collective, commands.roll, commands.yaw, commands.pitch)
        local rotorOutput = mixer:update()

        telemetryTimer = telemetryTimer + dt
        if telemetryTimer >= CONTROL.telemetry_dt then
            telemetryTimer = 0.0

            shared.telemetryTime = stateNow
            shared.telemetry = telemetry_builder.running({
                shared = shared,
                state = state,
                input = input,
                velocity = velocity,
                rotorOutput = rotorOutput,
                controllers = controller:pidControllers(),

                time = stateNow,
                dt = dt,
                stateAge = stateAge,
                yawRateAge = yawRateAge,
                velocityAge = velocityAge,
                inputAge = inputAge,
                inputStale = inputStale,

                collective = commands.collective,
                rollCmd = commands.roll,
                pitchCmd = commands.pitch,
                yawCmd = commands.yaw,

                targetHeight = terms.height.target,
                targetRoll = terms.roll.target,
                targetPitch = terms.pitch.target,
                targetYaw = terms.yaw.target,
                targetYawRate = terms.yaw.targetRate,

                yawRate = terms.yaw.rate,
                yawAngleActive = terms.yaw.angleActive,

                heightErr = terms.height.err,
                rollErr = terms.roll.err,
                pitchErr = terms.pitch.err,
                yawErr = terms.yaw.err,
                yawRateErr = terms.yaw.rateErr,
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
