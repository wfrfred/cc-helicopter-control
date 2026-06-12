local protocol = require("lib.protocol")
local config = require("config")

local telemetry_task = {}

local MODEM_SIDE = config.modem.side
local RECEIVE_TIMEOUT = config.telemetry.receive_timeout

function telemetry_task.run(shared)
    rednet.open(MODEM_SIDE)

    while shared.running do
        local sender, msg = rednet.receive(protocol.CONTROL.TELEMETRY, RECEIVE_TIMEOUT)

        if sender then
            if type(msg) == "table" then
                shared.telemetry = msg
                shared.telemetryTime = os.clock()
                shared.telemetrySender = sender
                shared.telemetryError = nil
            else
                shared.telemetryError = "bad telemetry message type: " .. type(msg)
            end
        end
    end
end

return telemetry_task
