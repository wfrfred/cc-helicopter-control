local mathx = require("lib.mathx")

local navigation = {}

local Navigator = {}
Navigator.__index = Navigator

local defaults = {
    arrival_radius = 5.0,
    waypoint_radius = 8.0,
    climb_tolerance = 1.0,
    altitude_tolerance = 1.0,
    heading_tolerance = math.rad(5),
    approach_distance = 40.0,
}

local function configValue(config, name)
    local value = config[name]

    if value ~= nil then
        return value
    end

    return defaults[name]
end

local function assertPosition(value, name)
    assert(type(value) == "table", name .. " must be a position table")
    assert(type(value.x) == "number", name .. ".x must be number")
    assert(type(value.y) == "number", name .. ".y must be number")
    assert(type(value.z) == "number", name .. ".z must be number")

    return value
end

local function copyPosition(value)
    return {
        x = value.x,
        y = value.y,
        z = value.z,
    }
end

local function horizontalTarget(value)
    return {
        x = value.x,
        z = value.z,
        y = value.y,
    }
end

local function horizontalDistance(a, b)
    local dx = b.x - a.x
    local dz = b.z - a.z

    return math.sqrt(dx * dx + dz * dz)
end

local function headingTo(from, to)
    return mathx.atan2(to.x - from.x, -(to.z - from.z))
end

local function waypointSummary(waypoint)
    if waypoint == nil then
        return nil
    end

    return {
        id = waypoint.id,
        name = waypoint.name or waypoint.id,
        position = copyPosition(waypoint.position),
    }
end

local function waypointList(waypoints)
    local out = {}

    for index, waypoint in ipairs(waypoints) do
        out[index] = waypointSummary(waypoint)
    end

    return out
end

local function approachSummary(approach)
    if approach == nil then
        return nil
    end

    return {
        id = approach.id,
        name = approach.name or approach.id,
    }
end

local function inactiveResult(self)
    return {
        active = false,
        phase = "idle",
        selected = waypointSummary(self.selected),
        waypoint = nil,
        approach = nil,
        leg = nil,
        target = nil,
        waypoints = waypointList(self.waypoints),
        arrived = false,
        reason = self.inactiveReason,
    }
end

local function findWaypoint(waypoints, id)
    for _, waypoint in ipairs(waypoints) do
        if waypoint.id == id then
            return waypoint
        end
    end

    return nil
end

local function routeStart(waypoint, approach, config)
    if approach == nil then
        return waypoint.position
    end

    if approach.path ~= nil and #approach.path > 0 then
        return assertPosition(approach.path[1], "approach.path[1]")
    end

    if approach.entry ~= nil then
        return assertPosition(approach.entry, "approach.entry")
    end

    assert(type(approach.heading) == "number", "approach.heading must be number when entry/path are omitted")

    local distance = approach.distance or configValue(config, "approach_distance")
    local destination = waypoint.position

    return {
        x = destination.x - math.sin(approach.heading) * distance,
        y = approach.altitude or waypoint.cruiseAltitude or destination.y,
        z = destination.z + math.cos(approach.heading) * distance,
    }
end

local function selectApproach(waypoint, position, config)
    local approaches = waypoint.approaches

    if approaches == nil or #approaches == 0 then
        return nil
    end

    local best = nil
    local bestCost = nil

    for _, approach in ipairs(approaches) do
        if approach.enabled ~= false then
            local start = routeStart(waypoint, approach, config)
            local cost = horizontalDistance(position, start)

            if bestCost == nil or cost < bestCost then
                best = approach
                bestCost = cost
            end
        end
    end

    assert(best ~= nil, "waypoint has approaches but none are enabled")

    return best
end

local function addLeg(legs, kind, position, heading, radius)
    assertPosition(position, "navigation leg.position")

    legs[#legs + 1] = {
        kind = kind,
        position = copyPosition(position),
        heading = heading,
        radius = radius,
    }
end

local function buildApproachLegs(waypoint, approach, config)
    local legs = {}
    local waypointRadius = waypoint.waypointRadius or configValue(config, "waypoint_radius")
    local arrivalRadius = waypoint.arrivalRadius or configValue(config, "arrival_radius")

    if approach ~= nil then
        if approach.path ~= nil then
            for index, point in ipairs(approach.path) do
                addLeg(legs, "route", assertPosition(point, "approach.path[" .. index .. "]"), nil, waypointRadius)
            end
        elseif approach.entry ~= nil then
            addLeg(legs, "entry", approach.entry, nil, waypointRadius)
        else
            addLeg(legs, "entry", routeStart(waypoint, approach, config), nil, waypointRadius)
        end

        local finalPosition = copyPosition(waypoint.position)

        if approach.finalAltitude ~= nil then
            finalPosition.y = approach.finalAltitude
        end

        addLeg(legs, "final", finalPosition, approach.heading, arrivalRadius)
    else
        addLeg(legs, "direct", waypoint.position, nil, arrivalRadius)
    end

    return legs
end

local function cruiseAltitude(waypoint, approach, currentHeight, legs)
    local finalAltitude = waypoint.position.y

    if approach ~= nil and approach.finalAltitude ~= nil then
        finalAltitude = approach.finalAltitude
    end

    if waypoint.hold ~= nil and waypoint.hold.altitude ~= nil then
        finalAltitude = waypoint.hold.altitude
    end

    local altitude = math.max(currentHeight, finalAltitude)

    if approach ~= nil and approach.cruiseAltitude ~= nil then
        altitude = math.max(altitude, approach.cruiseAltitude)
    end

    if waypoint.cruiseAltitude ~= nil then
        altitude = math.max(altitude, waypoint.cruiseAltitude)
    end

    for _, leg in ipairs(legs) do
        altitude = math.max(altitude, leg.position.y)
    end

    return altitude
end

local function currentLeg(active)
    return active.legs[active.legIndex]
end

local function legHeading(active, position)
    local leg = currentLeg(active)

    if leg.heading ~= nil then
        return mathx.wrapPi(leg.heading)
    end

    return mathx.wrapPi(headingTo(position, leg.position))
end

local function destinationHeight(active)
    local waypoint = active.waypoint
    local hold = waypoint.hold or {}

    return hold.altitude or active.destination.y
end

local function legHeight(active)
    local leg = currentLeg(active)

    if active.phase == "climb" or
        active.phase == "turn" or
        active.phase == "transit" or
        active.phase == "final_approach" then
        return active.cruiseAltitude
    end

    return leg.position.y or active.cruiseAltitude
end

local function targetForPhase(active, position, pose)
    if active.phase == "climb" then
        return {
            position = horizontalTarget(active.holdPosition),
            height = active.cruiseAltitude,
            heading = pose.heading,
        }
    end

    if active.phase == "turn" then
        return {
            position = horizontalTarget(active.holdPosition),
            height = active.cruiseAltitude,
            heading = legHeading(active, position),
        }
    end

    if active.phase == "arrived" then
        local waypoint = active.waypoint
        local hold = waypoint.hold or {}

        return {
            position = horizontalTarget(waypoint.position),
            height = destinationHeight(active),
            heading = hold.heading or active.arrivalHeading,
        }
    end

    if active.phase == "descend" then
        local waypoint = active.waypoint
        local hold = waypoint.hold or {}

        return {
            position = horizontalTarget(active.destination),
            height = destinationHeight(active),
            heading = hold.heading or active.arrivalHeading,
        }
    end

    return {
        position = horizontalTarget(currentLeg(active).position),
        height = legHeight(active),
        heading = legHeading(active, position),
    }
end

local function advanceLeg(active, position)
    if active.legIndex >= #active.legs then
        active.holdPosition = copyPosition(position)
        active.phase = "descend"
        return
    end

    active.legIndex = active.legIndex + 1
    active.holdPosition = copyPosition(position)
    active.phase = "turn"
end

local function updatePhase(active, position, pose, config)
    local climbTolerance = configValue(config, "climb_tolerance")
    local altitudeTolerance = configValue(config, "altitude_tolerance")
    local headingTolerance = configValue(config, "heading_tolerance")

    for _ = 1, 4 do
        if active.phase == "climb" then
            if position.y >= active.cruiseAltitude - climbTolerance then
                active.phase = "turn"
            else
                return
            end
        elseif active.phase == "turn" then
            if math.abs(mathx.wrapPi(legHeading(active, position) - pose.heading)) <= headingTolerance then
                local leg = currentLeg(active)
                active.phase = leg.kind == "final" and "final_approach" or "transit"
            else
                return
            end
        elseif active.phase == "transit" or active.phase == "final_approach" then
            local leg = currentLeg(active)

            if horizontalDistance(position, leg.position) <= leg.radius then
                advanceLeg(active, position)
            else
                return
            end
        elseif active.phase == "descend" then
            if math.abs(position.y - destinationHeight(active)) <= altitudeTolerance then
                active.phase = "arrived"
            else
                return
            end
        else
            return
        end
    end
end

local function activeResult(active, position, pose)
    local leg = currentLeg(active)

    return {
        active = true,
        phase = active.phase,
        selected = waypointSummary(active.waypoint),
        waypoint = waypointSummary(active.waypoint),
        approach = approachSummary(active.approach),
        leg = leg and {
            index = active.legIndex,
            count = #active.legs,
            kind = leg.kind,
            position = copyPosition(leg.position),
        } or nil,
        target = targetForPhase(active, position, pose),
        waypoints = waypointList(active.waypoints),
        arrived = active.phase == "arrived",
        reason = nil,
    }
end

function navigation.new(config)
    config = config or {}

    return setmetatable({
        config = config,
        waypoints = config.waypoints or {},
        selected = nil,
        active = nil,
        inactiveReason = nil,
    }, Navigator)
end

function Navigator:select(id)
    local waypoint = findWaypoint(self.waypoints, id)

    assert(waypoint ~= nil, "navigation waypoint not found: " .. tostring(id))
    assert(type(waypoint.id) == "string", "waypoint.id must be string")
    assertPosition(waypoint.position, "waypoint.position")

    if self.active ~= nil and self.active.waypoint.id ~= id then
        self.active = nil
    end

    self.selected = waypoint
    self.inactiveReason = "selected"

    return self:state()
end

function Navigator:activate(state, id)
    if id ~= nil then
        self:select(id)
    end

    assert(self.selected ~= nil, "navigation activation requires a selected waypoint")
    assert(type(state) == "table", "navigation activation requires state")

    local position = assertPosition(state.raw.position, "state.raw.position")
    local pose = state.body.pose
    local approach = selectApproach(self.selected, position, self.config)
    local legs = buildApproachLegs(self.selected, approach, self.config)

    self.active = {
        waypoint = self.selected,
        approach = approach,
        legs = legs,
        legIndex = 1,
        phase = "climb",
        holdPosition = copyPosition(position),
        destination = copyPosition(legs[#legs].position),
        cruiseAltitude = cruiseAltitude(self.selected, approach, position.y, legs),
        arrivalHeading = approach and approach.heading or pose.heading,
        waypoints = self.waypoints,
    }
    self.inactiveReason = nil

    return self:update(state, 0.0)
end

function Navigator:toggle(id, state)
    if self.selected ~= nil and self.selected.id == id and self.active == nil then
        return self:activate(state)
    end

    if self.active ~= nil and self.active.waypoint.id == id then
        return self:cancel("toggle")
    end

    return self:select(id)
end

function Navigator:cancel(reason)
    self.active = nil
    self.inactiveReason = reason or "cancelled"

    return self:state()
end

function Navigator:isActive()
    return self.active ~= nil
end

function Navigator:command(command, state)
    assert(type(command) == "table", "navigation command must be table")

    if command.action == "select" then
        return self:select(command.waypoint)
    end

    if command.action == "activate" then
        return self:activate(state, command.waypoint)
    end

    if command.action == "toggle" then
        return self:toggle(command.waypoint, state)
    end

    if command.action == "cancel" then
        return self:cancel("command")
    end

    error("unknown navigation command action: " .. tostring(command.action))
end

function Navigator:update(state, dt)
    if self.active == nil then
        return inactiveResult(self)
    end

    local position = assertPosition(state.raw.position, "state.raw.position")
    local pose = assert(type(state.body.pose) == "table" and state.body.pose, "state.body.pose must be table")

    updatePhase(self.active, position, pose, self.config)

    return activeResult(self.active, position, pose)
end

function Navigator:state()
    if self.active == nil then
        return inactiveResult(self)
    end

    return {
        active = true,
        phase = self.active.phase,
        selected = waypointSummary(self.active.waypoint),
        waypoint = waypointSummary(self.active.waypoint),
        approach = approachSummary(self.active.approach),
        leg = {
            index = self.active.legIndex,
            count = #self.active.legs,
            kind = currentLeg(self.active).kind,
            position = copyPosition(currentLeg(self.active).position),
        },
        target = nil,
        waypoints = waypointList(self.waypoints),
        arrived = self.active.phase == "arrived",
        reason = nil,
    }
end

return navigation
