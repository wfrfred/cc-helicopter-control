local mathx = require("lib.mathx")
local pid = require("pid")
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

local BASE_COLLECTIVE = CONTROL.base_collective

local COLLECTIVE_MIN = CONTROL.collective_min
local COLLECTIVE_MAX = CONTROL.collective_max

local heightPid = pid.new(CONTROL.pid.height)
local rollPid = pid.new(CONTROL.pid.roll)
local pitchPid = pid.new(CONTROL.pid.pitch)
local yawAnglePid = pid.new(CONTROL.pid.yaw_angle)
local yawRatePid = pid.new(CONTROL.pid.yaw_rate)

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

        local heightOut, heightErr = heightPid:update(targets.height, s.pos.y, dt)

        local rollErr = mathx.wrapPi(targets.roll - s.roll)
        local pitchErr = mathx.wrapPi(targets.pitch - s.pitch)

        local yawResult = yawLock:update(ctl.yaw, s.yaw, yawRate)
        local targetYawRate = yawResult.commanded_rate
        local yawErr = yawResult.yaw_err

        if yawResult.angle_active then
            targetYawRate = yawAnglePid:update(yawErr, 0.0, dt)
        end

        local rollCmd = rollPid:update(rollErr, 0.0, dt)
        local pitchCmd = pitchPid:update(pitchErr, 0.0, dt)
        local yawCmd, yawRateErr = yawRatePid:update(targetYawRate, yawRate, dt)

        local collective = mathx.clamp(
            BASE_COLLECTIVE + heightOut,
            COLLECTIVE_MIN,
            COLLECTIVE_MAX
        )

        mixer:set(collective, rollCmd, yawCmd, pitchCmd)
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
                controllers = {
                    height = heightPid,
                    roll = rollPid,
                    pitch = pitchPid,
                    yawAngle = yawAnglePid,
                    yawRate = yawRatePid,
                },

                time = stateNow,
                dt = dt,
                stateAge = stateAge,
                yawRateAge = yawRateAge,
                velocityAge = velocityAge,
                inputAge = inputAge,
                inputStale = inputStale,

                collective = collective,
                rollCmd = rollCmd,
                pitchCmd = pitchCmd,
                yawCmd = yawCmd,

                targetHeight = targets.height,
                targetRoll = targets.roll,
                targetPitch = targets.pitch,
                targetYaw = yawResult.target_yaw,
                targetYawRate = targetYawRate,

                yawRate = yawRate,
                yawAngleActive = yawResult.angle_active,

                heightErr = heightErr,
                rollErr = rollErr,
                pitchErr = pitchErr,
                yawErr = yawErr,
                yawRateErr = yawRateErr,
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
