local config = require("config")
local sync = config.runtime.sync

if sync.enabled == true then
    print("startup sync: " .. sync.target)

    local ok, err = pcall(function()
        shell.run("sync", sync.target)
    end)

    if not ok then
        print("startup sync failed: " .. tostring(err))
    end
else
    print("startup sync: disabled")
end

sleep(1)
term.clear()
term.setCursorPos(1, 1)
print("startup: flight controller")

local data_task = require("data_task")
local input_task = require("input_task")
local control_task = require("control_task")
local telemetry_task = require("telemetry_task")

assert(sublevel, "CC:Sable sublevel API not found")

local shared = {
    state = nil,
    stateTime = 0.0,
    yawRate = 0.0,
    yawRateTime = 0.0,
    velocity = nil,
    velocityTime = 0.0,
    input = input_task.defaultInput(),
    inputTime = 0.0,
    inputSender = nil,
    telemetry = nil,
    telemetryTime = 0.0,
    running = true,
}

parallel.waitForAny(
    function()
        data_task.run(shared)
    end,
    function()
        input_task.run(shared)
    end,
    function()
        control_task.run(shared)
    end,
    function()
        telemetry_task.run(shared)
    end
)
