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
    return {
        status = "running",
        time = data.time,
        dt = data.dt,

        stateAge = data.stateAge,
        yawRateAge = data.yawRateAge,
        velocityAge = data.velocityAge,

        inputAge = data.inputAge,
        inputStale = data.inputStale,
        inputSender = data.shared.inputSender,
        input = {
            roll = data.input.roll,
            pitch = data.input.pitch,
            yaw = data.input.yaw,
            climb = data.input.climb,
        },

        position = {
            x = data.state.pos.x,
            y = data.state.pos.y,
            z = data.state.pos.z,
        },

        output = {
            collective = data.collective,
            roll = data.rollCmd,
            pitch = data.pitchCmd,
            yaw = data.yawCmd,
            rotor = {
                upper = data.rotorOutput.upper,
                lower = data.rotorOutput.lower,
            },
        },

        pid = {
            height = pidTerms(data.controllers.height),
            roll = pidTerms(data.controllers.roll),
            pitch = pidTerms(data.controllers.pitch),
            yawAngle = data.yawAngleActive and pidTerms(data.controllers.yawAngle) or zeroPidTerms(),
            yawRate = pidTerms(data.controllers.yawRate),
        },

        target = {
            height = data.targetHeight,
            roll = data.targetRoll,
            pitch = data.targetPitch,
            yaw = data.targetYaw,
            yawRate = data.targetYawRate,
        },

        current = {
            height = data.state.pos.y,
            roll = data.state.roll,
            pitch = data.state.pitch,
            yaw = data.state.yaw,
            yawRate = data.yawRate,
            velocity = {
                x = data.velocity.x,
                y = data.velocity.y,
                z = data.velocity.z,
                total = data.velocity.total,
                horizontal = data.velocity.horizontal,
                vertical = data.velocity.vertical,
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
