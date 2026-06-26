local tablex = require("lib.tablex")

local terms = {}

function terms.waiting(input)
    local state = input.state
    local haveState = state ~= nil

    return {
        status = "waiting_sensors",
        time = input.now,
        havePose = haveState
            and state.body ~= nil
            and state.body.pose ~= nil
            and state.body.frame ~= nil
            and state.body.orientation ~= nil,
        haveRates = haveState
            and state.body ~= nil
            and state.body.angular ~= nil
            and state.body.angular.velocity ~= nil,
        haveVelocity = haveState
            and state.world ~= nil
            and state.world.velocity ~= nil,
    }
end

local function telemetryState(state)
    return {
        raw = {
            position = state.raw.position,
            velocity = state.raw.velocity,
            angularVelocity = state.raw.angularVelocity,
        },
        world = {
            position = state.world.position,
            velocity = state.world.velocity,
        },
        body = {
            frame = state.body.frame,
            pose = state.body.pose,
            velocity = state.body.velocity,
            angular = state.body.angular,
        },
        navigation = state.navigation,
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

local function navigationView(runtime, navigationConfig)
    runtime = runtime or {}

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
        waypoints = waypointCatalog(navigationConfig),
    }
end

local function heightView(input)
    local height = input.height or {}

    return {
        value = input.state.body.pose.height,
        target = height.target,
        rate = input.state.world.velocity.y,
        targetRate = height.rate or 0.0,
        error = height.error or 0.0,
    }
end

local function headingView(input)
    local heading = input.heading or {}

    return {
        angle = input.state.navigation.heading.angle,
        target = heading.target,
        rate = input.state.navigation.heading.rate,
        targetRate = heading.rate or 0.0,
        error = heading.error or 0.0,
    }
end

local function axisFields(axes, field)
    axes = axes or {}

    return {
        forward = axes.forward and axes.forward[field] or nil,
        right = axes.right and axes.right[field] or nil,
    }
end

local function pidAxes(axes)
    axes = axes or {}

    return {
        forward = axes.forward and axes.forward.pid or nil,
        right = axes.right and axes.right.pid or nil,
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
            return loop[field]
        end),
        rate = tablex.record.map(attitude.rate or {}, function(loop)
            return loop[field]
        end),
    })
end

local function attitudePidView(attitude)
    attitude = attitude or {}

    return tablex.record.map(attitude.angle or {}, function(angleLoop, axis)
        local rateLoop = attitude.rate and attitude.rate[axis] or {}

        return {
            angle = angleLoop.pid,
            rate = rateLoop.pid,
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
            error = vertical.position and vertical.position.error or nil,
        },
        velocity = {
            target = vertical.velocity and vertical.velocity.target or nil,
            current = vertical.velocity and vertical.velocity.current or nil,
            error = vertical.velocity and vertical.velocity.error or nil,
        },
        pid = {
            position = vertical.position and vertical.position.pid or nil,
            velocity = vertical.velocity and vertical.velocity.pid or nil,
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
    return {
        status = "running",
        time = input.now,
        dt = input.dt,
        age = {
            pose = input.now - input.state.time.pose,
            angularVelocity = input.now - input.state.time.angularVelocity,
            velocity = input.now - input.state.time.velocity,
        },
        input = {
            manual = input.input.manual,
            event = input.inputEvent,
            age = input.inputAge,
            stale = input.inputStale,
            sender = input.inputSender,
        },
        flight = input.flight,
        mode = input.mode,
        height = heightView(input),
        heading = headingView(input),
        state = telemetryState(input.state),
        control = controlView(input.control),
        navigation = navigationView(input.navigation, input.navigationConfig),
        command = input.command,
        rotor = input.rotor,
    }
end

return terms
