local input_protocol = {}

local function clamp(x, lo, hi)
    if x < lo then
        return lo
    end

    if x > hi then
        return hi
    end

    return x
end

local function axis(value)
    return clamp(value or 0.0, -1.0, 1.0)
end

local function navigationCommand(command)

    if command == nil then
        return nil
    end

    assert(type(command) == "table", "navigation command must be table")
    assert(type(command.action) == "string", "navigation command action must be string")

    if command.action == "cancel" then
        return {
            action = "cancel",
        }
    end

    assert(type(command.waypoint) == "string", "navigation command waypoint must be string")

    return {
        action = command.action,
        waypoint = command.waypoint,
    }
end

function input_protocol.defaultInput()
    return {
        manual = {
            mode = "manual.attitude",
            arm = true,
            attitude = {
                roll = 0.0,
                pitch = 0.0,
            },
            velocity = {
                forward = 0.0,
                right = 0.0,
                up = 0.0,
            },
            heading = {
                rate = 0.0,
            },
        },
        navigation = {
            action = nil,
            waypoint = nil,
        },
        event = {
            cruiseToggle = false,
            holdCapture = false,
        },
        seq = nil,
        time = nil,
    }
end

function input_protocol.decode(msg)
    if msg == nil then
        return input_protocol.defaultInput()
    end

    local event = msg.event or {}
    local manual = msg.manual or {}
    local attitude = manual.attitude or {}
    local velocity = manual.velocity or {}
    local heading = manual.heading or {}
    local command = navigationCommand(msg.navigation)

    return {
        manual = {
            mode = manual.mode or "manual.attitude",
            arm = manual.arm ~= false,
            attitude = {
                roll = axis(attitude.roll),
                pitch = axis(attitude.pitch),
            },
            velocity = {
                forward = axis(velocity.forward),
                right = axis(velocity.right),
                up = axis(velocity.up),
            },
            heading = {
                rate = axis(heading.rate),
            },
        },
        navigation = command or {
            action = nil,
            waypoint = nil,
        },
        event = {
            cruiseToggle = event.cruiseToggle == true,
            holdCapture = event.holdCapture == true,
        },
        seq = msg.seq,
        time = msg.time,
    }
end

return input_protocol
