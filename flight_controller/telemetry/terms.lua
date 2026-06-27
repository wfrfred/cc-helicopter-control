local mathx = require("lib.mathx")
local tablex = require("lib.tablex")

local terms = {}

local function frameView(frame)
    return {
        origin = frame.origin,
        qWorldFromLocal = frame.qWorldFromLocal,
        basis = frame:basis(),
    }
end

local function bodyAttitude(state)
    local basis = state.frames.body:basis()
    local forwardHorizontal = vector.new(basis.forward.x, 0.0, basis.forward.z)
    local horizontal = forwardHorizontal:length()

    return {
        roll = mathx.wrapPi(mathx.atan2(-basis.right.y, -basis.down.y)),
        pitch = mathx.wrapPi(mathx.atan2(basis.forward.y, horizontal)),
        heading = mathx.wrapPi(mathx.atan2(basis.forward.x, -basis.forward.z)),
    }
end

local function bodyRates(state)
    return {
        roll = state.body.angularVelocity.x,
        pitch = state.body.angularVelocity.y,
        yaw = state.body.angularVelocity.z,
    }
end

local function heading(state)
    local forward = state.frames.navigation:basis().forward

    return mathx.wrapPi(mathx.atan2(forward.x, -forward.z))
end

function terms.waiting(input)
    local state = input.state
    local haveState = state ~= nil

    return {
        status = "waiting_sensors",
        time = input.now,
        havePose = haveState
            and state.frames ~= nil
            and state.frames.body ~= nil
            and state.world ~= nil
            and state.world.position ~= nil,
        haveRates = haveState
            and state.body ~= nil
            and state.body.angularVelocity ~= nil,
        haveVelocity = haveState
            and state.world ~= nil
            and state.world.velocity ~= nil,
    }
end

local function telemetryState(state)
    return {
        frames = {
            world = frameView(state.frames.world),
            navigation = frameView(state.frames.navigation),
            body = frameView(state.frames.body),
        },
        world = {
            position = state.world.position,
            orientation = state.world.orientation,
            velocity = state.world.velocity,
            angularVelocity = state.world.angularVelocity,
        },
        navigation = {
            position = state.navigation.position,
            orientation = state.navigation.orientation,
            velocity = state.navigation.velocity,
            angularVelocity = state.navigation.angularVelocity,
        },
        body = {
            position = state.body.position,
            orientation = state.body.orientation,
            velocity = state.body.velocity,
            angularVelocity = state.body.angularVelocity,
            attitude = bodyAttitude(state),
            rates = bodyRates(state),
        },
    }
end

local function waypointCatalog(navigationConfig)
    local waypoints = navigationConfig and navigationConfig.waypoints or {}

    return tablex.list.map(waypoints, function(waypoint)
        return tablex.record.merge(tablex.record.pick(waypoint, { "id", "position" }), {
            name = waypoint.name or waypoint.id,
        })
    end)
end

local function modeView(input)
    return {
        name = input.modeResult.name,
        terms = input.modeResult.terms,
    }
end

local function navigationView(input, modeTerms)
    local runtime = input.modeResult.name == "navigation" and modeTerms or {}

    return {
        active = runtime.active == true,
        phase = runtime.phase or (runtime.active and "active" or "idle"),
        selected = runtime.selected,
        waypoint = runtime.waypoint,
        approach = runtime.approach,
        leg = runtime.leg,
        target = runtime.target,
        arrived = runtime.arrived,
        reason = runtime.reason,
        waypoints = waypointCatalog(input.navigationConfig),
    }
end

local function heightView(input, modeTerms)
    local height = modeTerms.height or {}

    return {
        value = -input.state.navigation.position.z,
        target = height.target,
        rate = -input.state.navigation.velocity.z,
        targetRate = height.rate or 0.0,
        error = height.error or 0.0,
    }
end

local function headingView(input, modeTerms)
    local headingTerms = modeTerms.heading or {}

    return {
        angle = heading(input.state),
        target = headingTerms.target,
        rate = input.state.navigation.angularVelocity.z,
        targetRate = headingTerms.rate or 0.0,
        error = headingTerms.error or 0.0,
    }
end

local function axisFields(axes, field)
    axes = axes or {}

    return {
        forward = axes.forward and (
            field == "error" and axes.forward.pid and axes.forward.pid.error or axes.forward[field]
        ) or nil,
        right = axes.right and (
            field == "error" and axes.right.pid and axes.right.pid.error or axes.right[field]
        ) or nil,
    }
end

local function displayPid(loop)
    if loop == nil or loop.pid == nil then
        return {
            p = 0.0,
            i = 0.0,
            d = 0.0,
            output = 0.0,
        }
    end

    return tablex.record.merge(loop.pid, {
        output = loop.output,
    })
end

local function pidAxes(axes)
    axes = axes or {}

    return {
        forward = displayPid(axes.forward),
        right = displayPid(axes.right),
    }
end

local function horizontalControlView(horizontal)
    if horizontal == nil or horizontal.kind ~= "position" then
        return horizontal
    end

    return {
        kind = horizontal.kind,
        position = {
            target = axisFields(horizontal.position, "target"),
            current = axisFields(horizontal.position, "current"),
            error = axisFields(horizontal.position, "error"),
        },
        velocity = {
            target = axisFields(horizontal.velocity, "target"),
            current = axisFields(horizontal.velocity, "current"),
            error = axisFields(horizontal.velocity, "error"),
        },
        output = horizontal.output,
        feedforward = horizontal.feedforward,
        pid = {
            position = pidAxes(horizontal.position),
            velocity = pidAxes(horizontal.velocity),
        },
    }
end

local function attitudeAxisFields(attitude, field)
    attitude = attitude or {}

    return tablex.record.transpose({ "roll", "pitch", "yaw" }, {
        angle = tablex.record.map(attitude.angle or {}, function(loop)
            return field == "error" and loop.pid and loop.pid.error or loop[field]
        end),
        rate = tablex.record.map(attitude.rate or {}, function(loop)
            return field == "error" and loop.pid and loop.pid.error or loop[field]
        end),
    })
end

local function attitudePidView(attitude)
    attitude = attitude or {}

    return tablex.record.map(attitude.angle or {}, function(angleLoop, axis)
        local rateLoop = attitude.rate and attitude.rate[axis] or {}

        return {
            angle = displayPid(angleLoop),
            rate = displayPid(rateLoop),
        }
    end)
end

local function attitudeControlView(attitude)
    if attitude == nil then
        return nil
    end

    return {
        target = tablex.record.merge({
            orientation = attitude.orientation,
        }, attitudeAxisFields(attitude, "target")),
        current = attitudeAxisFields(attitude, "current"),
        error = attitudeAxisFields(attitude, "error"),
        output = attitude.output,
        feedforward = attitude.feedforward,
        pid = attitudePidView(attitude),
    }
end

local function verticalControlView(vertical)
    if vertical == nil then
        return nil
    end

    return tablex.record.merge(vertical, {
        position = {
            target = vertical.position and vertical.position.target or nil,
            current = vertical.position and vertical.position.current or nil,
            error = vertical.position and vertical.position.pid and vertical.position.pid.error or nil,
        },
        velocity = {
            target = vertical.velocity and vertical.velocity.target or nil,
            current = vertical.velocity and vertical.velocity.current or nil,
            error = vertical.velocity and vertical.velocity.pid and vertical.velocity.pid.error or nil,
        },
        pid = {
            position = displayPid(vertical.position),
            velocity = displayPid(vertical.velocity),
        },
    })
end

local function controlView(control)
    control = control or {}

    return {
        horizontal = horizontalControlView(control.horizontal),
        vertical = verticalControlView(control.vertical),
        attitude = attitudeControlView(control.attitude),
        allocation = control.allocation,
    }
end

function terms.running(input)
    local modeTerms = input.modeResult.terms

    return {
        status = "running",
        time = input.now,
        dt = input.dt,
        age = {
            pose = input.now - input.state.sampleTime.pose,
            angularVelocity = input.now - input.state.sampleTime.angularVelocity,
            velocity = input.now - input.state.sampleTime.velocity,
        },
        input = {
            manual = input.input.manual,
            event = input.inputEvent,
            age = input.inputAge,
            stale = input.inputStale,
            sender = input.inputSender,
        },
        flight = input.flight,
        mode = modeView(input),
        height = heightView(input, modeTerms),
        heading = headingView(input, modeTerms),
        state = telemetryState(input.state),
        control = controlView(input.controlResult.terms),
        navigation = navigationView(input, modeTerms),
        command = input.controlResult.output,
        rotor = input.rotorResult.blades,
    }
end

return terms
