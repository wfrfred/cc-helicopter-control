local protocol = require("lib.protocol")

local actuator = {}

local Actuator = {}
Actuator.__index = Actuator

function actuator.new(hardware)
    rednet.open(hardware.modem_side)

    return setmetatable({}, Actuator)
end

function Actuator:send(output)
    rednet.broadcast(output.blades.upper, protocol.LAYER.UPPER)
    rednet.broadcast(output.blades.lower, protocol.LAYER.LOWER)
end

return actuator
