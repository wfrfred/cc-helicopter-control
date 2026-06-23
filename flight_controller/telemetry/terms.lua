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

local function copyTable(value)
    if type(value) ~= "table" then
        return value
    end

    local out = {}

    for key, child in pairs(value) do
        out[key] = copyTable(child)
    end

    return out
end

local function waypointCatalog(navigationConfig)
    local out = {}
    local waypoints = navigationConfig and navigationConfig.waypoints or {}

    for index, waypoint in ipairs(waypoints) do
        out[index] = {
            id = waypoint.id,
            name = waypoint.name or waypoint.id,
            position = copyTable(waypoint.position),
        }
    end

    return out
end

local function navigationView(runtime, navigationConfig)
    local view = copyTable(runtime or {})

    view.active = view.active == true
    view.phase = view.phase or (view.active and "active" or "idle")
    view.waypoints = waypointCatalog(navigationConfig)

    return view
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
        lock = {
            height = input.height.source,
            heading = input.heading.source,
        },
        height = input.height,
        heading = input.heading,
        state = telemetryState(input.state),
        control = input.control,
        navigation = navigationView(input.navigation, input.navigationConfig),
        command = input.command,
        rotor = input.rotor,
    }
end

return terms
