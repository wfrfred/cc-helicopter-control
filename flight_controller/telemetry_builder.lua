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

local function rawVelocityTotal(rawVelocity)
    return math.sqrt(
        rawVelocity.x * rawVelocity.x
            + rawVelocity.y * rawVelocity.y
            + rawVelocity.z * rawVelocity.z
    )
end

local function rawVelocityHorizontal(rawVelocity)
    return math.sqrt(rawVelocity.x * rawVelocity.x + rawVelocity.z * rawVelocity.z)
end

function telemetry_builder.running(data)
    local result = data.result
    local commands = result.commands
    local terms = result.terms
    local position = data.position
    local flight = data.flight
    local state = data.state
    local pose = state.body.pose
    local rawPosition = state.raw.position
    local velocity = state.body.velocity
    local rawVelocity = state.raw.velocity
    local yawAngleTerms

    if terms.yaw.angleActive then
        yawAngleTerms = pidTerms(data.controllers.yawAngle)
    else
        yawAngleTerms = zeroPidTerms()
    end

    return {
        status = "running",
        time = data.time,
        dt = data.dt,

        poseAge = data.poseAge,
        ratesAge = data.ratesAge,
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

        modes = {
            lateral = flight.mode.lateral,
            vertical = flight.mode.vertical,
            yaw = flight.mode.yaw,
        },

        position = {
            x = rawPosition.x,
            y = rawPosition.y,
            z = rawPosition.z,
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
            yawAngle = yawAngleTerms,
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
            height = rawPosition.y,
            verticalSpeed = -terms.verticalSpeed.current,
            roll = pose.roll,
            pitch = pose.pitch,
            yaw = pose.yaw,
            rollRate = terms.roll.rate,
            pitchRate = terms.pitch.rate,
            yawRate = terms.yaw.rate,
            velocity = {
                x = rawVelocity.x,
                y = rawVelocity.y,
                z = rawVelocity.z,
                total = rawVelocityTotal(rawVelocity),
                horizontal = rawVelocityHorizontal(rawVelocity),
                vertical = rawVelocity.y,
                forward = velocity.forward,
                right = velocity.right,
                down = velocity.down,
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
