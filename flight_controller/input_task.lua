local protocol = require("lib.protocol")
local config = require("config")

local input_task = {}

local function defaultInput()
    return {
        roll = 0.0,
        pitch = 0.0,
        yaw = 0.0,
        climb = 0.0,
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

local function normalize(msg)
    return {
        roll = axis(msg.roll),
        pitch = axis(msg.pitch),
        yaw = axis(msg.yaw),
        climb = axis(msg.climb),
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
            shared.input = normalize(msg)
            shared.inputTime = os.clock()
            shared.inputSender = sender
        end
    end
end

return input_task
