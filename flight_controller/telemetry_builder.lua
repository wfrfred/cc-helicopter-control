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
    local commands = data.commands
    local terms = data.terms
    local position = data.position

    return {
        status = "running",
        time = data.time,
        dt = data.dt,

        poseAge = data.poseAge,
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
            x = data.pose.pos.x,
            y = data.pose.pos.y,
            z = data.pose.pos.z,
        },

        output = {
            collective = commands.collective,
            collectiveFeedforward = terms.verticalSpeed.feedforward,
            collectiveFeedback = terms.verticalSpeed.feedback,
            roll = commands.roll,
            pitch = commands.pitch,
            yaw = commands.yaw,
            rotor = {
                upper = data.rotorOutput.upper,
                lower = data.rotorOutput.lower,
            },
        },

        pid = {
            height = pidTerms(data.controllers.height),
            verticalSpeed = pidTerms(data.controllers.verticalSpeed),
            positionRight = pidTerms(data.positionControllers.positionRight),
            positionForward = pidTerms(data.positionControllers.positionForward),
            velocityRight = pidTerms(data.positionControllers.velocityRight),
            velocityForward = pidTerms(data.positionControllers.velocityForward),
            roll = pidTerms(data.controllers.roll),
            pitch = pidTerms(data.controllers.pitch),
            yawAngle = terms.yaw.angleActive and pidTerms(data.controllers.yawAngle) or zeroPidTerms(),
            yawRate = pidTerms(data.controllers.yawRate),
        },

        target = {
            height = terms.height.target,
            verticalSpeed = terms.verticalSpeed.target,
            roll = terms.roll.target,
            pitch = terms.pitch.target,
            yaw = terms.yaw.target,
            yawRate = terms.yaw.targetRate,
        },

        current = {
            height = data.pose.pos.y,
            verticalSpeed = terms.verticalSpeed.current,
            roll = data.pose.roll,
            pitch = data.pose.pitch,
            yaw = data.pose.yaw,
            yawRate = terms.yaw.rate,
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
            height = terms.height.err,
            verticalSpeed = terms.verticalSpeed.err,
            roll = terms.roll.err,
            pitch = terms.pitch.err,
            yaw = terms.yaw.err,
            yawRate = terms.yaw.rateErr,
        },

        positionHold = {
            active = position.active,
            target = {
                x = position.targetX,
                z = position.targetZ,
            },
            targetVelocity = {
                right = position.targetVelocityRight,
                forward = position.targetVelocityForward,
            },
            currentVelocity = {
                right = position.currentVelocityRight,
                forward = position.currentVelocityForward,
            },
            error = {
                right = position.errorRight,
                forward = position.errorForward,
            },
            output = {
                right = position.outputRight,
                forward = position.outputForward,
                feedforwardRight = position.feedforwardRight,
                feedforwardForward = position.feedforwardForward,
                feedbackRight = position.feedbackRight,
                feedbackForward = position.feedbackForward,
                roll = position.roll,
                pitch = position.pitch,
            },
        },
    }
end

return telemetry_builder
