local protocol = require("lib.protocol")
local input_protocol = require("protocol.input")
local config = require("config")

local input_task = {}

function input_task.run(shared)
    rednet.open(config.runtime.modem.control)

    while shared.running do
        local sender, msg = rednet.receive(protocol.CONTROL.INPUT, config.runtime.input.receive_timeout)

        if sender then
            local input = input_protocol.decode(msg)

            if shared.input.event.cruiseToggle then
                input.event.cruiseToggle = true
            end

            if input.navigation.action ~= nil then
                shared.navigationCommand = input.navigation
                input.navigation = {
                    action = nil,
                    waypoint = nil,
                }
            end

            shared.input = input
            shared.inputTime = os.clock()
            shared.inputSender = sender
        end
    end
end

return input_task
