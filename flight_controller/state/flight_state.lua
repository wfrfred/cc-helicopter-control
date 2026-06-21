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

function flight_state.new()
    return setmetatable({}, State)
end

function State:update(input)
    if not ready(input.state) then
        return {
            name = "waiting_sensors",
            reason = "waiting_sensors",
        }
    end

    if input.inputStale then
        return {
            name = "running",
            reason = "input_stale_zeroed",
        }
    end

    return {
        name = "running",
        reason = "ready",
    }
end

return flight_state
