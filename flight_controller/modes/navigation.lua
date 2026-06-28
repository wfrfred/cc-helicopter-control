local controller = require("control.controller")
local frames = require("lib.frames")
local mathx = require("lib.mathx")
local tablex = require("lib.tablex")

local navigation = {}

local Navigation = {}
Navigation.__index = Navigation

local defaults = {
    arrival_radius = 5.0,
    waypoint_radius = 8.0,
    climb_tolerance = 1.0,
    altitude_tolerance = 1.0,
    heading_tolerance = math.rad(5),
    horizontal_speed_tolerance = 0.5,
    vertical_speed_tolerance = 0.3,
    heading_rate_tolerance = math.rad(3),
    approach_distance = 40.0,
}

local function configValue(config, name)
    local value = config[name]

    if value ~= nil then
        return value
    end

    return defaults[name]
end

local function heading(state)
    local forward = state.frames.navigation:basis().forward

    return mathx.wrapPi(mathx.atan2(forward.x, -forward.z))
end

local function assertFiniteNumber(name, value)
    assert(type(value) == "number", name .. " must be number")
    assert(value == value, name .. " must not be NaN")
    assert(value ~= math.huge and value ~= -math.huge, name .. " must be finite")
end

local function assertPosition(value, name)
    assert(type(value) == "table", name .. " must be a position table")
    assertFiniteNumber(name .. ".x", value.x)
    assertFiniteNumber(name .. ".y", value.y)
    assertFiniteNumber(name .. ".z", value.z)

    return value
end

local function horizontalPosition(value)
    return {
        x = value.x,
        z = value.z,
    }
end

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

local function horizontalDistance(a, b)
    return (horizontalVector(b) - horizontalVector(a)):length()
end

local function horizontalSpeed(velocity)
    return horizontalVector(velocity):length()
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
        position = tablex.record.copy(waypoint.position),
    }
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

local function inactiveTerms(self)
    return {
        active = false,
        phase = "idle",
        selected = waypointSummary(self.selected),
        waypoint = nil,
        approach = nil,
        leg = nil,
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
        position = tablex.record.copy(position),
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

        local finalPosition = tablex.record.copy(waypoint.position)

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

local function currentLeg(route)
    return route.legs[route.legIndex]
end

local function legHeading(route, position)
    local leg = currentLeg(route)

    if leg.heading ~= nil then
        return mathx.wrapPi(leg.heading)
    end

    return mathx.wrapPi(headingTo(position, leg.position))
end

local function destinationHeight(route)
    local waypoint = route.waypoint
    local hold = waypoint.hold or {}

    return hold.altitude or route.destination.y
end

local function legHeight(route)
    local leg = currentLeg(route)

    if route.phase == "climb" or
        route.phase == "turn" or
        route.phase == "transit" or
        route.phase == "final_approach" then
        return route.cruiseAltitude
    end

    return leg.position.y or route.cruiseAltitude
end

local function targetForPhase(route, position, pose)
    if route.phase == "climb" then
        return {
            position = horizontalPosition(route.holdPosition),
            height = route.cruiseAltitude,
            heading = pose.heading,
        }
    end

    if route.phase == "turn" then
        return {
            position = horizontalPosition(route.holdPosition),
            height = route.cruiseAltitude,
            heading = legHeading(route, position),
        }
    end

    if route.phase == "arrived" then
        local waypoint = route.waypoint
        local hold = waypoint.hold or {}

        return {
            position = horizontalPosition(waypoint.position),
            height = destinationHeight(route),
            heading = hold.heading or route.arrivalHeading,
        }
    end

    if route.phase == "descend" then
        local waypoint = route.waypoint
        local hold = waypoint.hold or {}

        return {
            position = horizontalPosition(route.destination),
            height = destinationHeight(route),
            heading = hold.heading or route.arrivalHeading,
        }
    end

    return {
        position = horizontalPosition(currentLeg(route).position),
        height = legHeight(route),
        heading = legHeading(route, position),
    }
end

local function advanceLeg(route, position)
    if route.legIndex >= #route.legs then
        route.holdPosition = tablex.record.copy(position)
        route.phase = "descend"
        return
    end

    route.legIndex = route.legIndex + 1
    route.holdPosition = tablex.record.copy(position)
    route.phase = "turn"
end

local function updatePhase(route, position, pose, motion, config)
    local climbTolerance = configValue(config, "climb_tolerance")
    local altitudeTolerance = configValue(config, "altitude_tolerance")
    local headingTolerance = configValue(config, "heading_tolerance")
    local horizontalSpeedTolerance = configValue(config, "horizontal_speed_tolerance")
    local verticalSpeedTolerance = configValue(config, "vertical_speed_tolerance")
    local headingRateTolerance = configValue(config, "heading_rate_tolerance")
    local horizontalStopped = horizontalSpeed(motion.worldVelocity) <= horizontalSpeedTolerance
    local verticalStopped = math.abs(motion.verticalSpeed) <= verticalSpeedTolerance
    local headingStopped = math.abs(motion.headingRate) <= headingRateTolerance

    for _ = 1, 4 do
        if route.phase == "climb" then
            if position.y >= route.cruiseAltitude - climbTolerance and verticalStopped then
                route.phase = "turn"
            else
                return
            end
        elseif route.phase == "turn" then
            if math.abs(mathx.wrapPi(legHeading(route, position) - pose.heading)) <= headingTolerance
                and headingStopped then
                local leg = currentLeg(route)
                route.phase = leg.kind == "final" and "final_approach" or "transit"
            else
                return
            end
        elseif route.phase == "transit" or route.phase == "final_approach" then
            local leg = currentLeg(route)

            if horizontalDistance(position, leg.position) <= leg.radius and horizontalStopped then
                advanceLeg(route, position)
            else
                return
            end
        elseif route.phase == "descend" then
            if math.abs(position.y - destinationHeight(route)) <= altitudeTolerance and verticalStopped then
                route.phase = "arrived"
            else
                return
            end
        else
            return
        end
    end
end

local function routeTerms(route)
    local leg = currentLeg(route)

    return {
        active = true,
        phase = route.phase,
        selected = waypointSummary(route.waypoint),
        waypoint = waypointSummary(route.waypoint),
        approach = approachSummary(route.approach),
        leg = leg and {
            index = route.legIndex,
            count = #route.legs,
            kind = leg.kind,
            position = tablex.record.copy(leg.position),
        } or nil,
        arrived = route.phase == "arrived",
        reason = nil,
    }
end

local function selectWaypoint(self, id)
    local waypoint = findWaypoint(self.waypoints, id)

    assert(waypoint ~= nil, "navigation waypoint not found: " .. tostring(id))
    assert(type(waypoint.id) == "string", "waypoint.id must be string")
    assertPosition(waypoint.position, "waypoint.position")

    if self.route ~= nil and self.route.waypoint.id ~= id then
        self.route = nil
    end

    self.selected = waypoint
    self.inactiveReason = "selected"
end

local function activateRoute(self, id, state)
    if id ~= nil then
        selectWaypoint(self, id)
    end

    assert(self.selected ~= nil, "navigation activation requires a selected waypoint")

    local position = state.world.position
    local currentHeading = heading(state)
    local approach = selectApproach(self.selected, position, self.config)
    local legs = buildApproachLegs(self.selected, approach, self.config)

    self.route = {
        waypoint = self.selected,
        approach = approach,
        legs = legs,
        legIndex = 1,
        phase = "climb",
        holdPosition = tablex.record.copy(position),
        destination = tablex.record.copy(legs[#legs].position),
        cruiseAltitude = cruiseAltitude(self.selected, approach, position.y, legs),
        arrivalHeading = approach and approach.heading or currentHeading,
    }
    self.inactiveReason = nil
end

local function cancelRoute(self)
    self.route = nil
    self.inactiveReason = "cancelled"
end

local function applyCommand(self, command, state)
    if command.action == "activate" then
        activateRoute(self, command.waypoint, state)
        return
    end

    error("navigation command action must be activate: " .. tostring(command.action))
end

local function targetForRoute(route, state)
    if route == nil then
        return nil
    end

    local position = state.world.position
    local attitude = {
        heading = heading(state),
    }

    return targetForPhase(route, position, attitude)
end

local function motion(state)
    return {
        worldVelocity = horizontalVector(state.world.velocity),
        verticalSpeed = state.navigation.velocity.z,
        headingRate = state.navigation.angularVelocity.z,
    }
end

local function buildTerms(self, state, phaseTarget)
    local route = self.route
    local terms = route == nil and inactiveTerms(self) or routeTerms(route)

    terms.target = phaseTarget

    if route ~= nil and phaseTarget ~= nil then
        terms.height = {
            target = phaseTarget.height,
            rate = 0.0,
            error = phaseTarget.height + state.navigation.position.z,
        }
        terms.heading = {
            target = phaseTarget.heading,
            rate = 0.0,
            error = mathx.wrapPi(phaseTarget.heading - heading(state)),
        }
    else
        terms.height = {
            target = -state.navigation.position.z,
            rate = 0.0,
            error = 0.0,
        }
        terms.heading = {
            target = heading(state),
            rate = 0.0,
            error = 0.0,
        }
    end

    return terms
end

local function buildTarget(ctx, phaseTarget)
    local positionError = vector.new(
        phaseTarget.position.x - ctx.state.world.position.x,
        0.0,
        phaseTarget.position.z - ctx.state.world.position.z
    )
    local position = frames.frdFromVector(
        frames.level(phaseTarget.heading):componentsOf(positionError)
    )
    local target = controller.target("position")

    target.horizontal.position.forward = position.forward
    target.horizontal.position.right = position.right
    target.vertical.position = -phaseTarget.height - ctx.state.navigation.position.z
    target.yaw.angle = phaseTarget.heading

    return target
end

function navigation.new(config)
    config = config or {}

    return setmetatable({
        config = config,
        waypoints = config.waypoints or {},
        selected = nil,
        route = nil,
        inactiveReason = nil,
    }, Navigation)
end

function Navigation:enter(ctx)
    local command = ctx.command

    if command == nil or command.action == nil then
        return
    end

    applyCommand(self, command, ctx.state)
end

function Navigation:update(ctx)
    assert(self.route ~= nil, "navigation target requires active route")

    local routeMotion = motion(ctx.state)
    local position = ctx.state.world.position
    local attitude = {
        heading = heading(ctx.state),
    }

    updatePhase(self.route, position, attitude, routeMotion, self.config)

    local phaseTarget = targetForRoute(self.route, ctx.state)

    return {
        target = buildTarget(ctx, phaseTarget),
        terms = buildTerms(self, ctx.state, phaseTarget),
    }
end

function Navigation:exit(ctx)
    if self.route ~= nil then
        cancelRoute(self)
    end
end

return navigation
