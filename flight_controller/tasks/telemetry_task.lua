local protocol = require("lib.protocol")
local config = require("config")

local telemetry_task = {}

function telemetry_task.run(shared)
    rednet.open(config.runtime.modem.control)

    while shared.running do
        if shared.telemetry then
            rednet.broadcast(shared.telemetry, protocol.CONTROL.TELEMETRY)
        end

        sleep(config.runtime.telemetry.broadcast_dt)
    end
end

return telemetry_task
