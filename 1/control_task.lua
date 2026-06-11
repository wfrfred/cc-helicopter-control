local mathx = require("mathx")
local pid = require("pid")
local rotor = require("rotor")
local config = require("config")

local control_task = {}

local LOOP_DT = config.control.loop_dt
local TELEMETRY_DT = config.control.telemetry_dt
local MAX_DT = config.control.max_dt
local INPUT_STALE_DT = config.control.input_stale_dt

local BASE_COLLECTIVE = config.control.base_collective
local HEIGHT_OUTPUT_SIGN = config.control.height_output_sign

local COLLECTIVE_MIN = config.control.collective_min
local COLLECTIVE_MAX = config.control.collective_max

local HOME_ROLL = config.control.home_roll
local HOME_PITCH = config.control.home_pitch

local MAX_TARGET_ROLL = config.control.max_target_roll
local MAX_TARGET_PITCH = config.control.max_target_pitch

local ROLL_TARGET_RATE = config.control.roll_target_rate
local PITCH_TARGET_RATE = config.control.pitch_target_rate
local YAW_TARGET_RATE = config.control.yaw_target_rate
local HEIGHT_TARGET_RATE = config.control.height_target_rate

local ROLL_CENTER_RATE = config.control.roll_center_rate
local PITCH_CENTER_RATE = config.control.pitch_center_rate

local YAW_LOCK_RATE_DEADBAND = config.control.yaw_lock_rate_deadband

local ROLL_OUTPUT_SIGN = config.control.roll_output_sign
local PITCH_OUTPUT_SIGN = config.control.pitch_output_sign
local YAW_OUTPUT_SIGN = config.control.yaw_output_sign

local heightPid = pid.new(config.control.pid.height)
local rollPid = pid.new(config.control.pid.roll)
local pitchPid = pid.new(config.control.pid.pitch)
local yawAnglePid = pid.new(config.control.pid.yaw_angle)
local yawRatePid = pid.new(config.control.pid.yaw_rate)

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

local function pidTerms(controller)
    local t = controller:terms()
    return {
        p = t.p,
        i = t.i,
        d = t.d,
        raw = t.raw,
        output = t.output,
    }
end

local function zeroPidTerms()
    return {
        p = 0.0,
        i = 0.0,
        d = 0.0,
        raw = 0.0,
        output = 0.0,
    }
end

local ZERO_INPUT = {
    roll = 0.0,
    pitch = 0.0,
    yaw = 0.0,
    climb = 0.0,
}

local function readInput(shared, now)
    local ctl = shared.input
    local inputTime = shared.inputTime or 0.0

    if type(ctl) ~= "table" or inputTime <= 0.0 then
        return ZERO_INPUT, nil, true
    end

    local inputAge = now - inputTime

    if inputAge > INPUT_STALE_DT then
        return ZERO_INPUT, inputAge, true
    end

    return ctl, inputAge, false
end

local function waitForState(shared)
    while shared.running and shared.state == nil do
        local now = os.clock()
        shared.telemetryTime = now
        shared.telemetry = {
            status = "waiting_state",
            time = now,
            dataError = shared.lastError,
            inputError = shared.inputError,
        }

        sleep(0.1)
    end
end

function control_task.run(shared)
    waitForState(shared)

    local initial = shared.state

    local targetHeight = initial.pos.y
    local targetRoll = HOME_ROLL
    local targetPitch = HOME_PITCH
    local targetYaw = initial.yaw

    local lastStateTime = shared.stateTime
    local lastPos = {
        x = initial.pos.x,
        y = initial.pos.y,
        z = initial.pos.z,
    }
    local velocity = {
        x = 0.0,
        y = 0.0,
        z = 0.0,
        total = 0.0,
        horizontal = 0.0,
        vertical = 0.0,
    }

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
        local stateTime = shared.stateTime or stateNow
        local stateAge = stateNow - stateTime
        local yawRate = shared.yawRate or 0.0
        local yawRateAge = 0.0
        if shared.yawRateTime and shared.yawRateTime > 0 then
            yawRateAge = stateNow - shared.yawRateTime
        end

        if stateTime > lastStateTime then
            local vdt = stateTime - lastStateTime

            if vdt > 0 then
                velocity.x = (s.pos.x - lastPos.x) / vdt
                velocity.y = (s.pos.y - lastPos.y) / vdt
                velocity.z = (s.pos.z - lastPos.z) / vdt
                velocity.horizontal = math.sqrt(velocity.x * velocity.x + velocity.z * velocity.z)
                velocity.vertical = velocity.y
                velocity.total = math.sqrt(
                    velocity.x * velocity.x +
                    velocity.y * velocity.y +
                    velocity.z * velocity.z
                )
            end

            lastPos = {
                x = s.pos.x,
                y = s.pos.y,
                z = s.pos.z,
            }
            lastStateTime = stateTime
        end

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

        rollCmd = ROLL_OUTPUT_SIGN * rollCmd
        pitchCmd = PITCH_OUTPUT_SIGN * pitchCmd
        yawCmd = YAW_OUTPUT_SIGN * yawCmd

        local collective = mathx.clamp(
            BASE_COLLECTIVE + HEIGHT_OUTPUT_SIGN * heightOut,
            COLLECTIVE_MIN,
            COLLECTIVE_MAX
        )

        rotor.set(collective, rollCmd, yawCmd, pitchCmd)
        local rotorOutput = rotor.update()

        telemetryTimer = telemetryTimer + dt
        if telemetryTimer >= TELEMETRY_DT then
            telemetryTimer = 0.0

            shared.telemetryTime = stateNow
            shared.telemetry = {
                status = "running",
                time = stateNow,
                dt = dt,

                stateAge = stateAge,
                yawRateAge = yawRateAge,

                inputAge = inputAge,
                inputStale = inputStale,
                inputSender = shared.inputSender,
                inputError = shared.inputError,
                input = {
                    roll = ctl.roll,
                    pitch = ctl.pitch,
                    yaw = ctl.yaw,
                    climb = ctl.climb,
                },

                position = {
                    x = s.pos.x,
                    y = s.pos.y,
                    z = s.pos.z,
                },

                output = {
                    collective = collective,
                    roll = rollCmd,
                    pitch = pitchCmd,
                    yaw = yawCmd,
                    rotor = {
                        upper = rotorOutput.upper,
                        lower = rotorOutput.lower,
                    },
                },

                pid = {
                    height = pidTerms(heightPid),
                    roll = pidTerms(rollPid),
                    pitch = pidTerms(pitchPid),
                    yawAngle = yawAngleActive and pidTerms(yawAnglePid) or zeroPidTerms(),
                    yawRate = pidTerms(yawRatePid),
                },

                target = {
                    height = targetHeight,
                    roll = targetRoll,
                    pitch = targetPitch,
                    yaw = targetYaw,
                    yawRate = targetYawRate,
                },

                current = {
                    height = s.pos.y,
                    roll = s.roll,
                    pitch = s.pitch,
                    yaw = s.yaw,
                    yawRate = yawRate,
                    velocity = {
                        x = velocity.x,
                        y = velocity.y,
                        z = velocity.z,
                        total = velocity.total,
                        horizontal = velocity.horizontal,
                        vertical = velocity.vertical,
                    },
                },

                error = {
                    height = heightErr,
                    roll = rollErr,
                    pitch = pitchErr,
                    yaw = yawErr,
                    yawRate = yawRateErr,
                },

                dataError = shared.lastError,
            }
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
