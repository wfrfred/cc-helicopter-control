local protocol = require("lib.protocol")
local input = require("input")
local config = require("config")

local input_task = {}

local MODEM_SIDE = config.modem.side
local SEND_DT = config.input.send_dt

local function defaultInput()
    return {
        controls = {
            roll = 0.0,
            pitch = 0.0,
            yaw = 0.0,
            climb = 0.0,
        },
        event = {
            cruiseLock = false,
        },
    }
end

function input_task.defaultInput()
    return defaultInput()
end

function input_task.run(shared)
    rednet.open(MODEM_SIDE)

    shared.input = shared.input or defaultInput()
    shared.inputTime = shared.inputTime or 0.0
    shared.inputSeq = shared.inputSeq or 0

    while shared.running do
        local now = os.clock()
        local ctl = input.read()

        shared.input = ctl
        shared.inputTime = now
        shared.inputSeq = shared.inputSeq + 1
        ctl.seq = shared.inputSeq
        ctl.time = now

        rednet.broadcast(ctl, protocol.CONTROL.INPUT)

        sleep(SEND_DT)
    end
end

return input_task
