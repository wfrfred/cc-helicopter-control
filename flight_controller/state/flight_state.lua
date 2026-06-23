local flight_state = {}

local State = {}
State.__index = State

local function ready(state)
    return state ~= nil
        and state.world ~= nil
        and state.world.position ~= nil
        and state.world.velocity ~= nil
        and state.body ~= nil
        and state.body.frame ~= nil
        and state.body.orientation ~= nil
        and state.body.pose ~= nil
        and state.body.angular ~= nil
        and state.body.angular.velocity ~= nil
        and state.navigation ~= nil
        and state.navigation.heading ~= nil
        and state.navigation.heading.angle ~= nil
        and state.navigation.heading.rate ~= nil
        and state.navigation.velocity ~= nil
        and state.time ~= nil
        and state.time.pose ~= nil
        and state.time.velocity ~= nil
        and state.time.angularVelocity ~= nil
end

local function sensorAges(state, now)
    return {
        pose = now - state.time.pose,
        velocity = now - state.time.velocity,
        angularVelocity = now - state.time.angularVelocity,
    }
end

local function maxAge(ages)
    return math.max(ages.pose, math.max(ages.velocity, ages.angularVelocity))
end

local function agePolicy(self, state, now)
    if now == nil or self.sensorAge == nil then
        return nil
    end

    local ages = sensorAges(state, now)
    local age = maxAge(ages)
    local status = "ready"

    if self.sensorAge.fault_dt ~= nil and age >= self.sensorAge.fault_dt then
        status = "sensor_age_fault"
    elseif self.sensorAge.warn_dt ~= nil and age >= self.sensorAge.warn_dt then
        status = "sensor_age_warning"
    end

    return {
        status = status,
        max = age,
        pose = ages.pose,
        velocity = ages.velocity,
        angularVelocity = ages.angularVelocity,
    }
end

function flight_state.new(sensorAge)
    return setmetatable({
        sensorAge = sensorAge,
    }, State)
end

function State:update(input)
    if not ready(input.state) then
        return {
            name = "waiting_sensors",
            reason = "waiting_sensors",
        }
    end

    local age = agePolicy(self, input.state, input.now)

    if input.inputStale then
        return {
            name = "running",
            reason = "input_stale_zeroed",
            sensorAge = age,
        }
    end

    return {
        name = "running",
        reason = age and age.status or "ready",
        sensorAge = age,
    }
end

return flight_state
