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

local function negatedPidTerms(controller)
    local t = controller:terms()
    return {
        p = -t.p,
        i = -t.i,
        d = -t.d,
        raw = -t.raw,
        output = -t.output,
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
    local rawPosition = data.rawPosition or {}
    local rawVelocity = data.rawVelocity or {}

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
            x = rawPosition.x or 0.0,
            y = rawPosition.y or 0.0,
            z = rawPosition.z or 0.0,
        },

        output = {
            collective = commands.collective,
            collectiveFeedforward = terms.verticalSpeed.feedforward,
            collectiveFeedback = terms.verticalSpeed.feedback,
            collectiveTiltCompensation = terms.verticalSpeed.tiltCompensation,
            collectiveTiltVerticalFactor = terms.verticalSpeed.tiltVerticalFactor,
            collectiveUncompensated = terms.verticalSpeed.uncompensatedOut,
            roll = commands.roll,
            pitch = commands.pitch,
            pitchFeedforward = terms.pitch.feedforward,
            pitchFeedback = terms.pitch.feedback,
            yaw = commands.yaw,
            yawFeedforward = terms.yaw.rateFeedforward,
            yawFeedback = terms.yaw.rateFeedback,
            rotor = {
                upper = data.rotorOutput.upper,
                lower = data.rotorOutput.lower,
            },
        },

        pid = {
            height = negatedPidTerms(data.controllers.height),
            verticalSpeed = negatedPidTerms(data.controllers.verticalSpeed),
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
            height = -terms.height.target,
            verticalSpeed = -terms.verticalSpeed.target,
            roll = terms.roll.target,
            pitch = terms.pitch.target,
            yaw = terms.yaw.target,
            yawRate = terms.yaw.targetRate,
        },

        current = {
            height = rawPosition.y or -terms.height.current,
            verticalSpeed = rawVelocity.vertical or -terms.verticalSpeed.current,
            roll = data.pose.roll,
            pitch = data.pose.pitch,
            yaw = data.pose.yaw,
            rollRate = terms.roll.rate,
            pitchRate = terms.pitch.rate,
            yawRate = terms.yaw.rate,
            velocity = {
                x = rawVelocity.x or 0.0,
                y = rawVelocity.y or 0.0,
                z = rawVelocity.z or 0.0,
                total = data.velocity.total,
                horizontal = data.velocity.horizontal,
                vertical = rawVelocity.vertical or -data.velocity.down,
                forward = data.velocity.forward,
                right = data.velocity.right,
                down = data.velocity.down,
            },
        },

        error = {
            height = -terms.height.err,
            verticalSpeed = -terms.verticalSpeed.err,
            roll = terms.roll.err,
            pitch = terms.pitch.err,
            yaw = terms.yaw.err,
            yawRate = terms.yaw.rateErr,
        },

        lock = {
            heightActive = terms.height.lockActive,
            heightPending = terms.height.lockPending,
            yawActive = terms.yaw.angleActive,
            yawPending = terms.yaw.anglePending,
        },

        positionHold = {
            active = position.active,
            target = {
                right = position.targetRight,
                forward = position.targetForward,
            },
            current = {
                right = position.currentPositionRight,
                forward = position.currentPositionForward,
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
