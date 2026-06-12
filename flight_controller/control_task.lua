local controller = require("controller")
local rotor = require("rotor")
local target_state = require("target_state")
local yaw_lock = require("yaw_lock")
local telemetry_builder = require("telemetry_builder")
local config = require("config")

local control_task = {}

local CONTROL = config.control

local LOOP_DT = CONTROL.loop_dt
local TELEMETRY_DT = CONTROL.telemetry_dt
local MAX_DT = CONTROL.max_dt
local INPUT_STALE_DT = CONTROL.input_stale_dt

local function clampDt(dt)
    if dt <= 0 then
        return LOOP_DT
    end

    return math.min(dt, MAX_DT)
end

local ZERO_INPUT = {
    roll = 0.0,
    pitch = 0.0,
    yaw = 0.0,
    climb = 0.0,
}

local function readInput(shared, now)
    local ctl = shared.input
    local inputAge = now - shared.inputTime

    if inputAge > INPUT_STALE_DT then
        return ZERO_INPUT, inputAge, true
    end

    return ctl, inputAge, false
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
    local ctrl = controller.new(CONTROL)

    local lastLoopTime = os.clock() - LOOP_DT
    local telemetryTimer = 0.0

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local ctl, inputAge, inputStale = readInput(shared, loopStart)
        targets:update(ctl, dt)

        local s = shared.state
        local stateNow = os.clock()
        local stateTime = shared.stateTime
        local stateAge = stateNow - stateTime
        local yawRate = shared.yawRate
        local velocity = shared.velocity
        local yawRateAge = stateNow - shared.yawRateTime
        local velocityAge = stateNow - shared.velocityTime

        local yawResult = yawLock:update(ctl.yaw, s.yaw, yawRate)

        local result = ctrl:update({
            targets = targets,
            state = s,
            yawRate = yawRate,
            velocity = velocity,
            yaw = yawResult,
            dt = dt,
        })

        local cmd = result.commands
        local dbg = result.debug

        mixer:set(cmd.collective, cmd.roll, cmd.yaw, cmd.pitch)
        local rotorOutput = mixer:update()

        telemetryTimer = telemetryTimer + dt
        if telemetryTimer >= TELEMETRY_DT then
            telemetryTimer = 0.0

            shared.telemetryTime = stateNow
            shared.telemetry = telemetry_builder.running({
                shared = shared,
                state = s,
                input = ctl,
                velocity = velocity,
                rotorOutput = rotorOutput,
                controllers = ctrl:pidControllers(),

                time = stateNow,
                dt = dt,
                stateAge = stateAge,
                yawRateAge = yawRateAge,
                velocityAge = velocityAge,
                inputAge = inputAge,
                inputStale = inputStale,

                collective = cmd.collective,
                rollCmd = cmd.roll,
                pitchCmd = cmd.pitch,
                yawCmd = cmd.yaw,

                targetHeight = dbg.height.target,
                targetRoll = dbg.roll.target,
                targetPitch = dbg.pitch.target,
                targetYaw = dbg.yaw.target,
                targetYawRate = dbg.yaw.targetRate,

                yawRate = dbg.yaw.rate,
                yawAngleActive = dbg.yaw.angleActive,

                heightErr = dbg.height.err,
                rollErr = dbg.roll.err,
                pitchErr = dbg.pitch.err,
                yawErr = dbg.yaw.err,
                yawRateErr = dbg.yaw.rateErr,
            })
        end

        local elapsed = os.clock() - loopStart
        local remain = LOOP_DT - elapsed

        if remain > 0 then
            sleep(remain)
        else
            sleep(0)
        end
    end
end

return control_task
