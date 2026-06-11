local protocol = require("protocol")
local config = require("config")

local telemetry_task = {}

local MODEM_SIDE = config.modem.control
local BROADCAST_DT = config.telemetry.broadcast_dt

function telemetry_task.run(shared)
    rednet.open(MODEM_SIDE)

    while shared.running do
        if shared.telemetry then
            rednet.broadcast(shared.telemetry, protocol.CONTROL.TELEMETRY)
        end

        sleep(BROADCAST_DT)
    end
end

return telemetry_task
