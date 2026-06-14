local Controller = require("controller")
local mathx = require("lib.mathx")
local rotor = require("rotor")
local target_state = require("target_state")
local navigation = require("navigation")
local position_hold = require("position_hold")
local rate_lock = require("rate_lock")
local config = require("config")

local control_task = {}

local function clampDt(dt)
    if dt <= 0 then
        return config.control.loop.dt
    end

    return math.min(dt, config.control.loop.max_dt)
end

local zeroInput = {
    roll = 0.0,
    pitch = 0.0,
    yaw = 0.0,
    climb = 0.0,
    event = {
        cruiseLock = false,
    },
}

local function readInput(shared, now)
    local input = shared.input
    local inputAge = now - shared.inputTime

    if inputAge > config.control.input.stale_dt then
        return zeroInput, inputAge, true
    end

    return input, inputAge, false
end

local function stateReady(state)
    return state ~= nil and
        state.raw.position ~= nil and
        state.raw.velocity ~= nil and
        state.body.pose ~= nil and
        state.body.rates ~= nil and
        state.body.velocity ~= nil and
        state.time.pose ~= nil and
        state.time.rates ~= nil and
        state.time.velocity ~= nil
end

local function waitForSensors(shared)
    while shared.running and not stateReady(shared.state) do
        local state = shared.state
        local haveState = state ~= nil
        local now = os.clock()

        shared.telemetryTime = now
        shared.telemetry = {
            status = "waiting_sensors",
            time = now,
            havePose = haveState and state.body.pose ~= nil,
            haveRates = haveState and state.body.rates ~= nil,
            haveVelocity = haveState and state.body.velocity ~= nil,
        }

        sleep(0.1)
    end
end

local lateralMode = {
    manual = "manual",
    cruise = "cruise",
    positionHold = "position_hold",
    navigation = "navigation",
}

local function manualLateralInput(input)
    return input.roll ~= 0 or input.pitch ~= 0
end

local function makeLateralMachine(initialState)
    return {
        mode = lateralMode.positionHold,
        positionTarget = navigation.makePositionTarget(initialState),
        cruiseVelocity = nil,
        cruiseManualReleasePending = false,
        navigationTarget = nil,
    }
end

local function enterLateralMode(machine, mode, state, positionHold)
    if machine.mode == mode then
        return
    end

    machine.mode = mode
    positionHold:reset()

    if mode == lateralMode.manual or mode == lateralMode.positionHold then
        machine.positionTarget = navigation.makePositionTarget(state)
    end
end

local function selectLateralMode(machine, input)
    if machine.navigationTarget ~= nil then
        return lateralMode.navigation
    end

    if machine.cruiseVelocity ~= nil then
        return lateralMode.cruise
    end

    if manualLateralInput(input) then
        return lateralMode.manual
    end

    return lateralMode.positionHold
end

local function updateCruiseTarget(machine, context)
    local manualInput = manualLateralInput(context.input)

    if context.input.event.cruiseLock then
        machine.cruiseVelocity = navigation.projectHorizontalVelocityToBodyFrd(context.state)
        machine.cruiseManualReleasePending = manualInput
        context.positionHold:reset()
        context.input.event.cruiseLock = false
        return
    end

    if machine.cruiseVelocity == nil then
        return
    end

    if machine.cruiseManualReleasePending then
        if not manualInput then
            machine.cruiseManualReleasePending = false
        end
        return
    end

    if manualInput then
        machine.cruiseVelocity = nil
    end
end

local function manualLateral(machine, context)
    return position_hold.inactive(), {
        roll = context.manualAttitude.roll,
        pitch = context.manualAttitude.pitch,
        source = machine.mode,
    }
end

local function cruiseLateral(machine, context)
    local positionResult = context.positionHold:updateVelocity(
        machine.cruiseVelocity,
        navigation.projectHorizontalVelocityToBodyFrd(context.state),
        context.dt
    )

    return positionResult, {
        roll = positionResult.roll,
        pitch = positionResult.pitch,
        source = machine.mode,
    }
end

local function positionHoldLateral(machine, context)
    local positionResult = context.positionHold:update(
        navigation.projectPositionTargetErrorToBodyFrd(machine.positionTarget, context.state),
        navigation.projectHorizontalVelocityToBodyFrd(context.state),
        context.dt
    )

    return positionResult, {
        roll = positionResult.roll,
        pitch = positionResult.pitch,
        source = machine.mode,
    }
end

local function navigationLateral(machine, context)
    local positionResult = context.positionHold:update(
        navigation.projectPositionTargetErrorToBodyFrd(machine.navigationTarget.position, context.state),
        navigation.projectHorizontalVelocityToBodyFrd(context.state),
        context.dt
    )

    return positionResult, {
        roll = positionResult.roll,
        pitch = positionResult.pitch,
        source = machine.mode,
    }
end

local function lateralPositionTarget(machine)
    if machine.mode == lateralMode.positionHold then
        return machine.positionTarget
    end

    if machine.mode == lateralMode.navigation then
        return machine.navigationTarget.position
    end

    return nil
end

local lateralHandlers = {
    manual = manualLateral,
    cruise = cruiseLateral,
    position_hold = positionHoldLateral,
    navigation = navigationLateral,
}

local function updateLateral(machine, context)
    updateCruiseTarget(machine, context)

    enterLateralMode(
        machine,
        selectLateralMode(machine, context.input),
        context.state,
        context.positionHold
    )

    return lateralHandlers[machine.mode](machine, context)
end

function control_task.run(shared)
    local mixer = rotor.new(config.hardware.rotor, config.calibration.rotor, config.calibration.mixer_axis)

    waitForSensors(shared)

    local initialState = shared.state
    local initial = initialState.body.pose

    local manualAttitude = target_state.new(initial, config.control)
    local positionHold = position_hold.new(config.control)
    local lateralMachine = makeLateralMachine(initialState)
    local heightLock = rate_lock.new({
        initial_target = initial.height,
        target_rate = config.control.vertical.target_rate,
        rate_deadband = config.control.vertical.lock.speed_deadband,
        relock_timeout = config.control.vertical.lock.relock_timeout,
    })
    local yawLock = rate_lock.new({
        initial_target = initial.yaw,
        target_rate = config.control.yaw.target_rate,
        rate_deadband = config.control.yaw.lock.rate_deadband,
        relock_timeout = config.control.yaw.lock.relock_timeout,
        error = function(target, current)
            return mathx.wrapPi(target - current)
        end,
    })
    local controller = Controller.new(config.control)
    local controllerPids = controller:pidControllers()
    local positionPids = positionHold:pidControllers()
    local flight = {
        mode = {},
        target = {},
        lock = {},
    }

    local lastLoopTime = os.clock() - config.control.loop.dt
    local telemetryTimer = 0.0

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local input, inputAge, inputStale = readInput(shared, loopStart)
        local inputEvent = {
            cruiseLock = input.event.cruiseLock,
        }

        manualAttitude:update(input, dt)

        local now = os.clock()
        local state = shared.state
        local pose = state.body.pose
        local rates = state.body.rates
        local height = pose.height
        local verticalSpeed = state.raw.velocity.y
        local poseAge = now - state.time.pose
        local ratesAge = now - state.time.rates
        local velocityAge = now - state.time.velocity

        local positionResult, attitudeTarget = updateLateral(lateralMachine, {
            input = input,
            state = state,
            manualAttitude = manualAttitude,
            positionHold = positionHold,
            dt = dt,
        })

        local verticalLock = heightLock:update(input.climb, height, verticalSpeed, dt)
        local yawLockResult = yawLock:update(input.yaw, pose.yaw, rates.yaw, dt)

        flight.mode.lateral = lateralMachine.mode
        flight.mode.vertical = "height"
        flight.mode.yaw = "yaw"
        flight.lock.height = verticalLock.state
        flight.lock.yaw = yawLockResult.state
        flight.target.attitude = attitudeTarget
        flight.target.position = lateralPositionTarget(lateralMachine)
        flight.target.vertical = {
            height = verticalLock.target,
            rate = verticalLock.commandedRate,
            active = verticalLock.active,
            pending = verticalLock.pending,
            error = verticalLock.error,
            source = verticalLock.state,
        }
        flight.target.yaw = {
            angle = yawLockResult.target,
            rate = yawLockResult.commandedRate,
            active = yawLockResult.active,
            pending = yawLockResult.pending,
            error = yawLockResult.error,
            source = yawLockResult.state,
        }

        local result = controller:update({
            target = flight.target,
            state = {
                pose = state.body.pose,
                rates = state.body.rates,
                height = height,
                verticalSpeed = verticalSpeed,
            },
            dt = dt,
        })

        mixer:setCommands(result.commands)
        local rotorOutput = mixer:update()

        shared.target = flight.target
        shared.controlResult = result
        shared.commands = result.commands

        telemetryTimer = telemetryTimer + dt
        if telemetryTimer >= config.control.loop.telemetry_dt then
            telemetryTimer = 0.0

            local terms = result.terms
            local rawPosition = state.raw.position
            local rawVelocity = state.raw.velocity
            local bodyVelocity = state.body.velocity
            shared.telemetryTime = now
            shared.telemetry = {
                status = "running",
                time = now,
                dt = dt,

                age = {
                    pose = poseAge,
                    rates = ratesAge,
                    velocity = velocityAge,
                },

                input = {
                    controls = {
                        roll = input.roll,
                        pitch = input.pitch,
                        yaw = input.yaw,
                        climb = input.climb,
                    },
                    event = {
                        cruiseLock = inputEvent.cruiseLock,
                    },
                    age = inputAge,
                    stale = inputStale,
                    sender = shared.inputSender,
                },

                mode = {
                    lateral = flight.mode.lateral,
                    vertical = flight.mode.vertical,
                    yaw = flight.mode.yaw,
                },

                lock = {
                    height = flight.lock.height,
                    yaw = flight.lock.yaw,
                },

                state = {
                    raw = {
                        position = {
                            x = rawPosition.x,
                            y = rawPosition.y,
                            z = rawPosition.z,
                        },
                        velocity = {
                            x = rawVelocity.x,
                            y = rawVelocity.y,
                            z = rawVelocity.z,
                        },
                    },
                    body = {
                        pose = {
                            height = pose.height,
                            roll = pose.roll,
                            pitch = pose.pitch,
                            yaw = pose.yaw,
                        },
                        velocity = {
                            forward = bodyVelocity.forward,
                            right = bodyVelocity.right,
                            down = bodyVelocity.down,
                        },
                        rates = {
                            roll = rates.roll,
                            pitch = rates.pitch,
                            yaw = rates.yaw,
                        },
                    },
                },

                output = {
                    commands = result.commands,
                    collective = {
                        command = result.commands.collective,
                        feedforward = terms.verticalSpeed.feedforward,
                        feedback = terms.verticalSpeed.feedback,
                        uncompensated = terms.verticalSpeed.uncompensatedOut,
                        tilt = {
                            compensation = terms.verticalSpeed.tiltCompensation,
                            verticalFactor = terms.verticalSpeed.tiltVerticalFactor,
                        },
                    },
                    pitch = {
                        command = result.commands.pitch,
                        feedforward = terms.pitch.feedforward,
                        feedback = terms.pitch.feedback,
                    },
                    yaw = {
                        command = result.commands.yaw,
                        feedforward = terms.yaw.rateFeedforward,
                        feedback = terms.yaw.rateFeedback,
                    },
                    rotor = {
                        upper = rotorOutput.upper,
                        lower = rotorOutput.lower,
                    },
                },

                pid = {
                    vertical = {
                        height = controllerPids.height:terms(),
                        speed = controllerPids.verticalSpeed:terms(),
                    },
                    position = {
                        right = positionPids.positionRight:terms(),
                        forward = positionPids.positionForward:terms(),
                    },
                    velocity = {
                        right = positionPids.velocityRight:terms(),
                        forward = positionPids.velocityForward:terms(),
                    },
                    attitude = {
                        roll = controllerPids.roll:terms(),
                        pitch = controllerPids.pitch:terms(),
                    },
                    yaw = {
                        angle = controllerPids.yawAngle:terms(),
                        rate = controllerPids.yawRate:terms(),
                    },
                },

                target = {
                    vertical = {
                        height = terms.height.target,
                        speed = terms.verticalSpeed.target,
                    },
                    attitude = {
                        roll = terms.roll.target,
                        pitch = terms.pitch.target,
                    },
                    yaw = {
                        angle = terms.yaw.target,
                        rate = terms.yaw.targetRate,
                    },
                },

                current = {
                    vertical = {
                        height = terms.height.current,
                        speed = terms.verticalSpeed.current,
                    },
                    attitude = {
                        roll = terms.roll.current,
                        pitch = terms.pitch.current,
                    },
                    yaw = {
                        angle = terms.yaw.current,
                        rate = terms.yaw.rate,
                    },
                },

                error = {
                    vertical = {
                        height = terms.height.err,
                        speed = terms.verticalSpeed.err,
                    },
                    attitude = {
                        roll = terms.roll.err,
                        pitch = terms.pitch.err,
                    },
                    yaw = {
                        angle = terms.yaw.err,
                        rate = terms.yaw.rateErr,
                    },
                },

                positionHold = {
                    active = positionResult.active,
                    position = {
                        target = {
                            right = positionResult.targetRight,
                            forward = positionResult.targetForward,
                        },
                        current = {
                            right = positionResult.currentPositionRight,
                            forward = positionResult.currentPositionForward,
                        },
                        error = {
                            right = positionResult.errorRight,
                            forward = positionResult.errorForward,
                        },
                    },
                    velocity = {
                        target = {
                            right = positionResult.targetVelocityRight,
                            forward = positionResult.targetVelocityForward,
                        },
                        current = {
                            right = positionResult.currentVelocityRight,
                            forward = positionResult.currentVelocityForward,
                        },
                        error = {
                            right = positionResult.velocityErrorRight,
                            forward = positionResult.velocityErrorForward,
                        },
                    },
                    output = {
                        right = {
                            value = positionResult.outputRight,
                            feedforward = positionResult.feedforwardRight,
                            feedback = positionResult.feedbackRight,
                        },
                        forward = {
                            value = positionResult.outputForward,
                            feedforward = positionResult.feedforwardForward,
                            feedback = positionResult.feedbackForward,
                        },
                        attitude = {
                            roll = positionResult.roll,
                            pitch = positionResult.pitch,
                        },
                    },
                },
            }
        end

        local elapsed = os.clock() - loopStart
        local remain = config.control.loop.dt - elapsed

        if remain > 0 then
            sleep(remain)
        else
            sleep(0)
        end
    end
end

return control_task
