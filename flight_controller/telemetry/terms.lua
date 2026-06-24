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
    local out = {}
    local waypoints = navigationConfig and navigationConfig.waypoints or {}

    for index, waypoint in ipairs(waypoints) do
        out[index] = {
            id = waypoint.id,
            name = waypoint.name or waypoint.id,
            position = waypoint.position,
        }
    end

    return out
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
        control = input.control,
        navigation = navigationView(input.navigation, input.navigationConfig),
        command = input.command,
        rotor = input.rotor,
    }
end

return terms
