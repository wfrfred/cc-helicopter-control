local mathx = require("lib.mathx")
local pid = require("pid")
local rotor = require("rotor")
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

local HOME_ROLL = CONTROL.home_roll
local HOME_PITCH = CONTROL.home_pitch

local MAX_TARGET_ROLL = CONTROL.max_target_roll
local MAX_TARGET_PITCH = CONTROL.max_target_pitch

local ROLL_TARGET_RATE = CONTROL.roll_target_rate
local PITCH_TARGET_RATE = CONTROL.pitch_target_rate
local YAW_TARGET_RATE = CONTROL.yaw_target_rate
local HEIGHT_TARGET_RATE = CONTROL.height_target_rate

local ROLL_CENTER_RATE = CONTROL.roll_center_rate
local PITCH_CENTER_RATE = CONTROL.pitch_center_rate

local YAW_LOCK_RATE_DEADBAND = CONTROL.yaw_lock_rate_deadband

local heightPid = pid.new(CONTROL.pid.height)
local rollPid = pid.new(CONTROL.pid.roll)
local pitchPid = pid.new(CONTROL.pid.pitch)
local yawAnglePid = pid.new(CONTROL.pid.yaw_angle)
local yawRatePid = pid.new(CONTROL.pid.yaw_rate)

local function moveToward(x, target, rate, dt)
    local d = target - x
    local step = rate * dt

    if math.abs(d) <= step then
        return target
    end

    if d > 0 then
        return x + step
    end

    return x - step
end

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

    local targetHeight = initial.pos.y
    local targetRoll = HOME_ROLL
    local targetPitch = HOME_PITCH
    local targetYaw = initial.yaw

    local lastLoopTime = os.clock() - LOOP_DT
    local telemetryTimer = 0.0
    local yawWasManual = false
    local yawLockPending = false

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local ctl, inputAge, inputStale = readInput(shared, loopStart)

        if ctl.roll ~= 0 then
            targetRoll = mathx.clamp(
                targetRoll + ctl.roll * ROLL_TARGET_RATE * dt,
                -MAX_TARGET_ROLL,
                MAX_TARGET_ROLL
            )
        else
            targetRoll = moveToward(targetRoll, HOME_ROLL, ROLL_CENTER_RATE, dt)
        end

        if ctl.pitch ~= 0 then
            targetPitch = mathx.clamp(
                targetPitch + ctl.pitch * PITCH_TARGET_RATE * dt,
                -MAX_TARGET_PITCH,
                MAX_TARGET_PITCH
            )
        else
            targetPitch = moveToward(targetPitch, HOME_PITCH, PITCH_CENTER_RATE, dt)
        end

        targetHeight = targetHeight + ctl.climb * HEIGHT_TARGET_RATE * dt

        local s = shared.state
        local stateNow = os.clock()
        local stateTime = shared.stateTime
        local stateAge = stateNow - stateTime
        local yawRate = shared.yawRate
        local velocity = shared.velocity
        local yawRateAge = stateNow - shared.yawRateTime
        local velocityAge = stateNow - shared.velocityTime

        local heightOut, heightErr = heightPid:update(targetHeight, s.pos.y, dt)

        local rollErr = mathx.wrapPi(targetRoll - s.roll)
        local pitchErr = mathx.wrapPi(targetPitch - s.pitch)
        local yawManual = ctl.yaw ~= 0
        local targetYawRate
        local yawErr
        local yawAngleActive = false

        if yawManual then
            targetYaw = s.yaw
            yawErr = 0.0
            targetYawRate = ctl.yaw * YAW_TARGET_RATE
            yawLockPending = false
        else
            if yawWasManual then
                yawLockPending = true
            end

            if yawLockPending then
                yawErr = 0.0
                targetYawRate = 0.0

                if math.abs(yawRate) < YAW_LOCK_RATE_DEADBAND then
                    targetYaw = s.yaw
                    yawLockPending = false
                end
            else
                yawErr = mathx.wrapPi(targetYaw - s.yaw)
                targetYawRate = yawAnglePid:update(yawErr, 0.0, dt)
                yawAngleActive = true
            end
        end

        yawWasManual = yawManual

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

                targetHeight = targetHeight,
                targetRoll = targetRoll,
                targetPitch = targetPitch,
                targetYaw = targetYaw,
                targetYawRate = targetYawRate,

                yawRate = yawRate,
                yawAngleActive = yawAngleActive,

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
