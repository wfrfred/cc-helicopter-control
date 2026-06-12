local protocol = require("lib.protocol")
local config = require("config")

local input_task = {}

local MODEM_SIDE = config.runtime.modem.control
local RECEIVE_TIMEOUT = config.runtime.input.receive_timeout

local function defaultInput()
    return {
        roll = 0.0,
        pitch = 0.0,
        yaw = 0.0,
        climb = 0.0,
    }
end

local function axis(value)
    return protocol.clamp(value, -1.0, 1.0)
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
    rednet.open(MODEM_SIDE)

    while shared.running do
        local sender, msg = rednet.receive(protocol.CONTROL.INPUT, RECEIVE_TIMEOUT)

        if sender then
            shared.input = normalize(msg)
            shared.inputTime = os.clock()
            shared.inputSender = sender
        end
    end
end

return input_task
