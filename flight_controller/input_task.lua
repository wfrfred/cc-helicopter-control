local protocol = require("lib.protocol")
local config = require("config")

local input_task = {}

local function defaultInput()
    return {
        controls = {
            roll = 0.0,
            pitch = 0.0,
            heading = 0.0,
            climb = 0.0,
        },
        event = {
            cruiseLock = false,
            navigation = nil,
        },
    }
end

local function clamp(x, lo, hi)
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

local function axis(value)
    return clamp(value, -1.0, 1.0)
end

local function navigationCommand(event)
    local command = event and event.navigation

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

local function normalize(msg)
    local controls = msg.controls
    local event = msg.event or {}

    return {
        controls = {
            roll = axis(controls.roll),
            pitch = axis(controls.pitch),
            heading = axis(controls.heading or 0.0),
            climb = axis(controls.climb),
        },
        event = {
            cruiseLock = event.cruiseLock == true,
            navigation = navigationCommand(event),
        },
        seq = msg.seq,
        time = msg.time,
    }
end

function input_task.defaultInput()
    return defaultInput()
end

function input_task.run(shared)
    rednet.open(config.runtime.modem.control)

    while shared.running do
        local sender, msg = rednet.receive(protocol.CONTROL.INPUT, config.runtime.input.receive_timeout)

        if sender then
            local input = normalize(msg)

            if shared.input.event.cruiseLock then
                input.event.cruiseLock = true
            end

            if input.event.navigation ~= nil then
                shared.navigationCommand = input.event.navigation
                input.event.navigation = nil
            end

            shared.input = input
            shared.inputTime = os.clock()
            shared.inputSender = sender
        end
    end
end

return input_task
