local Controller = require("controller")
local attitude_decoupler = require("lib.attitude_decoupler")
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
            havePose = haveState and state.body.pose ~= nil and state.body.frame ~= nil,
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
    return controls.roll ~= 0 or controls.pitch ~= 0
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

local function allocateCommands(result, control)
    local rawCommands = copyCommands(result.commands)
    local decoupledCommands = attitude_decoupler.apply(control.attitude_decoupler, rawCommands)
    local finalCommands = finalClampCommands(decoupledCommands, control.output_limits)
    local attitude = result.output.attitude

    result.commands = finalCommands
    result.output.commands = finalCommands
    result.output.rawCommands = rawCommands
    result.output.decoupledCommands = decoupledCommands
    result.output.finalCommands = finalCommands

    attitude.roll.controllerCommand = rawCommands.roll
    attitude.pitch.controllerCommand = rawCommands.pitch
    attitude.yaw.controllerCommand = rawCommands.yaw

    attitude.roll.decoupledCommand = decoupledCommands.roll
    attitude.pitch.decoupledCommand = decoupledCommands.pitch
    attitude.yaw.decoupledCommand = decoupledCommands.yaw

    attitude.roll.command = finalCommands.roll
    attitude.pitch.command = finalCommands.pitch
    attitude.yaw.command = finalCommands.yaw
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

local function updateNavigationCommand(machine, command, state)
    if command == nil then
        return machine.navigator:state()
    end

    local result = machine.navigator:command(command, state)

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
    local navigationResult = machine.navigator:update(context.state, context.dt)

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
    local navigationResult = updateNavigationCommand(machine, context.navigationCommand, context.state)

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
        pose = pose,
        rates = state.body.rates,
        vertical = {
            height = pose.height,
            speed = state.raw.velocity.y,
        },
    }
end

local function makeTarget(attitude, position, verticalLock, headingLockResult)
    return {
        attitude = attitude,
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
            angle = headingLockResult.target,
            rate = headingLockResult.commandedRate,
            active = headingLockResult.active,
            pending = headingLockResult.pending,
            error = headingLockResult.error,
            source = headingLockResult.state,
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
        target = mathx.wrapPi(targetHeading),
        error = mathx.wrapPi(targetHeading - pose.heading),
        commandedRate = 0.0,
        active = true,
        pending = false,
        state = "navigation_" .. navigationResult.phase,
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
    local headingLock = rate_lock.new({
        initial_target = initial.heading,
        target_rate = config.control.heading.target_rate,
        rate_deadband = config.control.heading.lock.rate_deadband,
        relock_timeout = config.control.heading.lock.relock_timeout,
        error = function(target, current)
            return mathx.wrapPi(target - current)
        end,
    })
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
        local headingLockResult = headingLock:update(controls.heading, pose.heading, headingRate, dt)
        local attitudeHeading = pose.heading

        if headingLockResult.active then
            attitudeHeading = headingLockResult.target
        end

        local navigationCommand = shared.navigationCommand
        shared.navigationCommand = nil

        local navigationWasActive = lateralMachine.navigator:isActive()
        local positionResult, attitudeTarget, navigationResult = updateLateral(lateralMachine, {
            input = input,
            state = state,
            manualAttitude = manualAttitude,
            positionHold = positionHold,
            attitudeHeading = attitudeHeading,
            navigationCommand = navigationCommand,
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
            headingLockResult = lockedRateTarget(headingLock, pose.heading, headingRate)
            attitudeHeading = headingLockResult.target
        elseif navigationHeading ~= nil then
            headingLockResult = navigationHeading
            attitudeHeading = headingLockResult.target
        end

        local target = makeTarget(
            attitudeTarget,
            lateralPositionTarget(lateralMachine, navigationResult),
            verticalLock,
            headingLockResult
        )

        local result = controller:update({
            target = target,
            state = controlState,
            dt = dt,
        })

        allocateCommands(result, config.control)

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
                    heading = headingLockResult.state,
                },

                state = makeTelemetryState(state),

                output = result.output,

                pid = {
                    vertical = {
                        height = controllerPids.vertical.height:terms(),
                        speed = controllerPids.vertical.speed:terms(),
                    },
                    position = {
                        x = positionPids.positionX:terms(),
                        z = positionPids.positionZ:terms(),
                    },
                    velocity = {
                        x = positionPids.velocityX:terms(),
                        z = positionPids.velocityZ:terms(),
                    },
                    attitude = {
                        roll = {
                            angle = controllerPids.attitude.roll.angle:terms(),
                            rate = controllerPids.attitude.roll.rate:terms(),
                        },
                        pitch = {
                            angle = controllerPids.attitude.pitch.angle:terms(),
                            rate = controllerPids.attitude.pitch.rate:terms(),
                        },
                        yaw = {
                            angle = controllerPids.attitude.yaw.angle:terms(),
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
