local telemetry_builder = {}

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

function telemetry_builder.running(data)
    local shared = data.shared
    local state = data.state
    local input = data.input
    local velocity = data.velocity
    local rotorOutput = data.rotorOutput
    local controllers = data.controllers

    return {
        status = "running",
        time = data.time,
        dt = data.dt,

        stateAge = data.stateAge,
        yawRateAge = data.yawRateAge,
        velocityAge = data.velocityAge,

        inputAge = data.inputAge,
        inputStale = data.inputStale,
        inputSender = shared.inputSender,
        input = {
            roll = input.roll,
            pitch = input.pitch,
            yaw = input.yaw,
            climb = input.climb,
        },

        position = {
            x = state.pos.x,
            y = state.pos.y,
            z = state.pos.z,
        },

        output = {
            collective = data.collective,
            roll = data.rollCmd,
            pitch = data.pitchCmd,
            yaw = data.yawCmd,
            rotor = {
                upper = rotorOutput.upper,
                lower = rotorOutput.lower,
            },
        },

        pid = {
            height = pidTerms(controllers.height),
            roll = pidTerms(controllers.roll),
            pitch = pidTerms(controllers.pitch),
            yawAngle = data.yawAngleActive and pidTerms(controllers.yawAngle) or zeroPidTerms(),
            yawRate = pidTerms(controllers.yawRate),
        },

        target = {
            height = data.targetHeight,
            roll = data.targetRoll,
            pitch = data.targetPitch,
            yaw = data.targetYaw,
            yawRate = data.targetYawRate,
        },

        current = {
            height = state.pos.y,
            roll = state.roll,
            pitch = state.pitch,
            yaw = state.yaw,
            yawRate = data.yawRate,
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
            height = data.heightErr,
            roll = data.rollErr,
            pitch = data.pitchErr,
            yaw = data.yawErr,
            yawRate = data.yawRateErr,
        },
    }
end

return telemetry_builder
