local Controller = require("controller")
local attitude_allocator = require("lib.attitude_allocator")
local attitude_math = require("lib.attitude_math")
local mathx = require("lib.mathx")
local rotor = require("rotor")
local target_state = require("target_state")
local position_hold = require("position_hold")
local navigation = require("navigation")
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
    controls = {
        roll = 0.0,
        pitch = 0.0,
        heading = 0.0,
        climb = 0.0,
    },
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
        state.body.frame ~= nil and
        state.body.orientation ~= nil and
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
            havePose = haveState and state.body.pose ~= nil and state.body.frame ~= nil
                and state.body.orientation ~= nil,
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

local verticalMode = {
    heightHold = "height_hold",
}

local headingMode = {
    headingHold = "heading_hold",
}

local function manualLateralInput(controls)
    return controls.roll ~= 0 or controls.pitch ~= 0 or controls.heading ~= 0
end

local positionTargetAxes = {
    x = { x = 1.0 },
    z = { z = 1.0 },
}

local function makePositionTarget(state)
    return mathx.project(state.raw.position, positionTargetAxes)
end

local function worldPositionError(target, state)
    local position = state.raw.position

    return {
        x = target.x - position.x,
        z = target.z - position.z,
    }
end

local function worldHorizontalVelocity(state)
    return {
        x = state.raw.velocity.x,
        z = state.raw.velocity.z,
    }
end

local function copyCommands(commands)
    return {
        collective = commands.collective,
        roll = commands.roll,
        pitch = commands.pitch,
        yaw = commands.yaw,
    }
end

local function finalClampCommands(commands, limits)
    return {
        collective = commands.collective,
        roll = mathx.clamp(commands.roll, limits.roll_min, limits.roll_max),
        pitch = mathx.clamp(commands.pitch, limits.pitch_min, limits.pitch_max),
        yaw = mathx.clamp(commands.yaw, limits.yaw_min, limits.yaw_max),
    }
end

local function allocateCommands(result, control, pose)
    local rawCommands = copyCommands(result.commands)
    local allocated = attitude_allocator.apply(control.attitude_allocator, pose, rawCommands)
    local finalCommands = finalClampCommands(allocated.commands, control.output_limits)
    local attitude = result.output.attitude

    result.commands = finalCommands
    result.output.commands = finalCommands
    result.output.rawCommands = rawCommands
    result.output.allocatedCommands = allocated.commands
    result.output.finalCommands = finalCommands
    result.output.attitudeAllocator = allocated.debug

    attitude.roll.controllerCommand = rawCommands.roll
    attitude.pitch.controllerCommand = rawCommands.pitch
    attitude.yaw.controllerCommand = rawCommands.yaw

    attitude.roll.allocatedCommand = allocated.commands.roll
    attitude.pitch.allocatedCommand = allocated.commands.pitch
    attitude.yaw.allocatedCommand = allocated.commands.yaw

    attitude.roll.command = finalCommands.roll
    attitude.pitch.command = finalCommands.pitch
    attitude.yaw.command = finalCommands.yaw
end

local function attachHeadingTelemetry(result, target, pose)
    result.target.commandedAttitude = {
        roll = target.attitude.roll,
        pitch = target.attitude.pitch,
        heading = target.heading.angle,
        source = target.attitude.source,
    }
    result.target.heading = target.heading
    result.current.heading = {
        angle = pose.heading,
    }
    result.error.heading = {
        angle = target.heading.error,
    }
end

local function headingRateFromAttitudeRates(bodyFrame, rates)
    local forward = bodyFrame.forward
    local horizontal = forward.x * forward.x + forward.z * forward.z

    if horizontal < 1.0e-6 then
        return 0.0
    end

    local function fromForwardChange(x, z)
        return (-forward.z * x + forward.x * z) / horizontal
    end

    return (rates.pitch or 0.0) * fromForwardChange(-bodyFrame.down.x, -bodyFrame.down.z)
        + (rates.yaw or 0.0) * fromForwardChange(bodyFrame.right.x, bodyFrame.right.z)
end

local function makeHeadingLock(initial)
    return {
        target = mathx.wrapPi(initial.heading),
        wasManual = false,
        pending = false,
        pendingTime = 0.0,
    }
end

local function headingLookaheadAngle(control)
    local tau = control.attitude.time_constant

    assert(tau > 0.0, "attitude time_constant must be positive for heading lookahead")

    return control.heading.lookahead_rate * tau
end

local function makeHeadingTarget(target, pose, state, active, pending)
    local angle = mathx.wrapPi(target)

    return {
        angle = angle,
        active = active,
        pending = pending,
        error = mathx.wrapPi(angle - pose.heading),
        source = state,
    }
end

local function captureHeadingLock(lock, heading)
    lock.target = mathx.wrapPi(heading)
    lock.wasManual = false
    lock.pending = false
    lock.pendingTime = 0.0
end

local function lockedHeadingTarget(lock, pose)
    captureHeadingLock(lock, pose.heading)

    return makeHeadingTarget(lock.target, pose, "locked", true, false)
end

local function updateHeadingLock(lock, controls, pose, headingRate, dt, control)
    if controls.heading ~= 0.0 then
        lock.target = mathx.wrapPi(pose.heading)
        lock.wasManual = true
        lock.pending = false
        lock.pendingTime = 0.0

        return makeHeadingTarget(
            pose.heading + controls.heading * headingLookaheadAngle(control),
            pose,
            "manual_lookahead",
            true,
            false
        )
    end

    if lock.wasManual then
        lock.pending = true
        lock.pendingTime = 0.0
        lock.wasManual = false
    end

    if lock.pending then
        lock.pendingTime = lock.pendingTime + dt

        local stopped = math.abs(headingRate) < control.heading.lock.rate_deadband
        local timedOut = control.heading.lock.relock_timeout > 0.0
            and lock.pendingTime >= control.heading.lock.relock_timeout

        if stopped or timedOut then
            captureHeadingLock(lock, pose.heading)
        else
            return makeHeadingTarget(pose.heading, pose, "pending", false, true)
        end
    end

    return makeHeadingTarget(lock.target, pose, "locked", true, false)
end

local function makeLateralMachine(initialState, navigator)
    return {
        mode = lateralMode.positionHold,
        positionTarget = makePositionTarget(initialState),
        cruiseWorldVelocity = nil,
        cruiseManualReleasePending = false,
        navigator = navigator,
    }
end

local function enterLateralMode(machine, mode, state, positionHold)
    if machine.mode == mode then
        return
    end

    machine.mode = mode
    positionHold:reset()

    if mode == lateralMode.manual or mode == lateralMode.positionHold then
        machine.positionTarget = makePositionTarget(state)
    end
end

local function selectLateralMode(machine, controls)
    if machine.navigator:isActive() then
        return lateralMode.navigation
    end

    if machine.cruiseWorldVelocity ~= nil then
        return lateralMode.cruise
    end

    if manualLateralInput(controls) then
        return lateralMode.manual
    end

    return lateralMode.positionHold
end

local function navigationMotion(state, headingRate)
    return {
        worldVelocity = worldHorizontalVelocity(state),
        verticalSpeed = state.raw.velocity.y,
        headingRate = headingRate,
    }
end

local function updateNavigationCommand(machine, command, state, motion)
    if command == nil then
        return machine.navigator:state()
    end

    local result = machine.navigator:command(command, state, motion)

    if result.active then
        machine.cruiseWorldVelocity = nil
    end

    return result
end

local function cancelNavigationForManualInput(machine, controls)
    if not machine.navigator:isActive() then
        return false
    end

    if manualLateralInput(controls) or controls.climb ~= 0.0 or controls.heading ~= 0.0 then
        machine.navigator:cancel("manual")
        return true
    end

    return false
end

local function updateCruiseTarget(machine, context)
    local manualInput = manualLateralInput(context.input.controls)

    if context.input.event.cruiseLock then
        machine.cruiseWorldVelocity = worldHorizontalVelocity(context.state)
        machine.cruiseManualReleasePending = manualInput
        context.positionHold:reset()
        context.input.event.cruiseLock = false
        return
    end

    if machine.cruiseWorldVelocity == nil then
        return
    end

    if machine.cruiseManualReleasePending then
        if not manualInput then
            machine.cruiseManualReleasePending = false
        end
        return
    end

    if manualInput then
        machine.cruiseWorldVelocity = nil
    end
end

local function attitudeTarget(source, attitude)
    return {
        roll = attitude.roll,
        pitch = attitude.pitch,
        source = source,
    }
end

local function manualLateral(machine, context)
    return position_hold.inactive(), context.manualAttitude:target(machine.mode)
end

local function cruiseLateral(machine, context)
    local positionResult = context.positionHold:updateVelocity(
        machine.cruiseWorldVelocity,
        worldHorizontalVelocity(context.state),
        context.attitudeHeading,
        context.dt
    )

    return positionResult, attitudeTarget(machine.mode, positionResult.output.attitude)
end

local function positionHoldLateral(machine, context)
    local positionResult = context.positionHold:update(
        worldPositionError(machine.positionTarget, context.state),
        worldHorizontalVelocity(context.state),
        context.attitudeHeading,
        context.dt
    )

    return positionResult, attitudeTarget(machine.mode, positionResult.output.attitude)
end

local function navigationLateral(machine, context)
    local navigationResult = machine.navigator:update(context.state, context.dt, context.navigationMotion)

    if not navigationResult.active then
        return position_hold.inactive(), context.manualAttitude:target(machine.mode), navigationResult
    end

    local positionResult = context.positionHold:update(
        worldPositionError(navigationResult.target.position, context.state),
        worldHorizontalVelocity(context.state),
        navigationResult.target.heading,
        context.dt
    )

    return positionResult, attitudeTarget(machine.mode, positionResult.output.attitude), navigationResult
end

local function lateralPositionTarget(machine, navigationResult)
    if machine.mode == lateralMode.positionHold then
        return machine.positionTarget
    end

    if machine.mode == lateralMode.navigation and
        navigationResult ~= nil and
        navigationResult.target ~= nil then
        return navigationResult.target.position
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
    local navigationResult = updateNavigationCommand(
        machine,
        context.navigationCommand,
        context.state,
        context.navigationMotion
    )

    if cancelNavigationForManualInput(machine, context.input.controls) then
        navigationResult = machine.navigator:state()
    end

    updateCruiseTarget(machine, context)

    enterLateralMode(
        machine,
        selectLateralMode(machine, context.input.controls),
        context.state,
        context.positionHold
    )

    local positionResult, target, handlerNavigationResult = lateralHandlers[machine.mode](machine, context)

    return positionResult, target, handlerNavigationResult or navigationResult
end

local function makeControlState(state)
    local pose = state.body.pose

    return {
        bodyFrame = state.body.frame,
        orientation = state.body.orientation,
        pose = pose,
        rates = state.body.rates,
        vertical = {
            height = pose.height,
            speed = state.raw.velocity.y,
        },
    }
end

local function makeTarget(
    attitude,
    position,
    verticalLock,
    headingLockResult,
    attitudeHeading,
    currentFrame
)
    local fullFrame = attitude_math.frameFromPose(
        attitude.roll,
        attitude.pitch,
        attitudeHeading
    )
    local qFull = attitude_math.quaternionFromFrame(fullFrame):normalize()
    local redFrame = attitude_math.reducedFrameFromTargetDown(currentFrame, fullFrame)
    local qRed = attitude_math.quaternionFromFrame(redFrame):normalize()
    local yawPriority = mathx.clamp(config.control.heading.yaw_priority, 0.0, 1.0)
    local qMixed = qRed:slerp(qFull, yawPriority):normalize()
    local targetAttitude = {
        roll = attitude.roll,
        pitch = attitude.pitch,
        source = attitude.source,
        orientation = qMixed,
        fullOrientation = qFull,
        reducedOrientation = qRed,
        yawPriority = yawPriority,
    }

    return {
        attitude = targetAttitude,
        position = position,
        vertical = {
            height = verticalLock.target,
            speed = verticalLock.commandedRate,
            active = verticalLock.active,
            pending = verticalLock.pending,
            error = verticalLock.error,
            source = verticalLock.state,
        },
        heading = {
            angle = headingLockResult.angle,
            active = headingLockResult.active,
            pending = headingLockResult.pending,
            error = headingLockResult.error,
            source = headingLockResult.source,
        },
    }
end

local function navigationVerticalLock(navigationResult, vertical)
    if not navigationResult.active or navigationResult.target == nil then
        return nil
    end

    local targetHeight = navigationResult.target.height

    if targetHeight == nil then
        return nil
    end

    return {
        target = targetHeight,
        error = targetHeight - vertical.height,
        commandedRate = 0.0,
        active = true,
        pending = false,
        state = "navigation_" .. navigationResult.phase,
    }
end

local function navigationHeadingLock(navigationResult, pose)
    if not navigationResult.active or navigationResult.target == nil then
        return nil
    end

    local targetHeading = navigationResult.target.heading

    if targetHeading == nil then
        return nil
    end

    return {
        angle = mathx.wrapPi(targetHeading),
        error = mathx.wrapPi(targetHeading - pose.heading),
        active = true,
        pending = false,
        source = "navigation_" .. navigationResult.phase,
    }
end

local function lockedRateTarget(lock, currentValue, measuredRate)
    lock:capture(currentValue)

    return lock:update(0.0, currentValue, measuredRate, 0.0)
end

local function makeTelemetryState(state)
    return {
        raw = {
            position = state.raw.position,
            velocity = state.raw.velocity,
        },
        body = {
            frame = state.body.frame,
            pose = state.body.pose,
            velocity = state.body.velocity,
            rates = state.body.rates,
        },
    }
end

function control_task.run(shared)
    local mixer = rotor.new(config.hardware.rotor, config.calibration.rotor)

    waitForSensors(shared)

    local initialState = shared.state
    local initial = initialState.body.pose

    local manualAttitude = target_state.new(initial, config.control)
    local positionHold = position_hold.new(config.control)
    local navigator = navigation.new(config.navigation)
    local lateralMachine = makeLateralMachine(initialState, navigator)
    local heightLock = rate_lock.new({
        initial_target = initial.height,
        target_rate = config.control.vertical.target_rate,
        rate_deadband = config.control.vertical.lock.speed_deadband,
        relock_timeout = config.control.vertical.lock.relock_timeout,
    })
    local headingLock = makeHeadingLock(initial)
    local controller = Controller.new(config.control)
    local controllerPids = controller:pidControllers()
    local positionPids = positionHold:pidControllers()

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

        manualAttitude:update(input.controls, dt)

        local now = os.clock()
        local state = shared.state
        local pose = state.body.pose
        local controlState = makeControlState(state)
        local headingRate = headingRateFromAttitudeRates(controlState.bodyFrame, controlState.rates)
        local controls = input.controls
        local vertical = controlState.vertical
        local verticalLock = heightLock:update(controls.climb, vertical.height, vertical.speed, dt)
        local headingLockResult = updateHeadingLock(
            headingLock,
            controls,
            pose,
            headingRate,
            dt,
            config.control
        )
        local attitudeHeading = headingLockResult.angle

        local navigationCommand = shared.navigationCommand
        shared.navigationCommand = nil

        local navigationWasActive = lateralMachine.navigator:isActive()
        local motion = navigationMotion(state, headingRate)
        local positionResult, attitudeTarget, navigationResult = updateLateral(lateralMachine, {
            input = input,
            state = state,
            manualAttitude = manualAttitude,
            positionHold = positionHold,
            attitudeHeading = attitudeHeading,
            navigationCommand = navigationCommand,
            navigationMotion = motion,
            dt = dt,
        })
        local navigationExited = navigationWasActive and not lateralMachine.navigator:isActive()
        local navigationVertical = navigationVerticalLock(navigationResult, vertical)
        local navigationHeading = navigationHeadingLock(navigationResult, pose)

        if navigationExited and controls.climb == 0.0 then
            verticalLock = lockedRateTarget(heightLock, vertical.height, vertical.speed)
        elseif navigationVertical ~= nil then
            verticalLock = navigationVertical
        end

        if navigationExited and controls.heading == 0.0 then
            headingLockResult = lockedHeadingTarget(headingLock, pose)
            attitudeHeading = headingLockResult.angle
        elseif navigationHeading ~= nil then
            headingLockResult = navigationHeading
            attitudeHeading = headingLockResult.angle
        end

        local target = makeTarget(
            attitudeTarget,
            lateralPositionTarget(lateralMachine, navigationResult),
            verticalLock,
            headingLockResult,
            attitudeHeading,
            controlState.bodyFrame
        )

        local result = controller:update({
            target = {
                attitude = target.attitude,
                vertical = target.vertical,
            },
            state = controlState,
            dt = dt,
        })

        attachHeadingTelemetry(result, target, pose)
        allocateCommands(result, config.control, controlState.pose)

        mixer:setCommands(result.commands)
        local rotorOutput = mixer:update()
        result.output.rotor = rotorOutput.blades

        shared.target = target
        shared.controlResult = result
        shared.commands = result.commands

        telemetryTimer = telemetryTimer + dt
        if telemetryTimer >= config.control.loop.telemetry_dt then
            telemetryTimer = 0.0

            shared.telemetryTime = now
            shared.telemetry = {
                status = "running",
                time = now,
                dt = dt,

                age = {
                    pose = now - state.time.pose,
                    rates = now - state.time.rates,
                    velocity = now - state.time.velocity,
                },

                input = {
                    controls = input.controls,
                    event = inputEvent,
                    age = inputAge,
                    stale = inputStale,
                    sender = shared.inputSender,
                },

                mode = {
                    lateral = lateralMachine.mode,
                    vertical = verticalMode.heightHold,
                    heading = headingMode.headingHold,
                },

                lock = {
                    height = verticalLock.state,
                    heading = headingLockResult.source,
                },

                state = makeTelemetryState(state),

                output = result.output,

                pid = {
                    vertical = {
                        height = controllerPids.vertical.height:terms(),
                        speed = controllerPids.vertical.speed:terms(),
                    },
                    position = {
                        forward = positionPids.positionForward:terms(),
                        right = positionPids.positionRight:terms(),
                    },
                    velocity = {
                        forward = positionPids.velocityForward:terms(),
                        right = positionPids.velocityRight:terms(),
                    },
                    attitude = {
                        roll = {
                            rate = controllerPids.attitude.roll.rate:terms(),
                        },
                        pitch = {
                            rate = controllerPids.attitude.pitch.rate:terms(),
                        },
                        yaw = {
                            rate = controllerPids.attitude.yaw.rate:terms(),
                        },
                    },
                },

                target = result.target,

                current = result.current,

                error = result.error,

                positionHold = positionResult,
                navigation = navigationResult,
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
