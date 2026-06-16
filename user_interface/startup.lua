local config = require("config")
local sync = config.sync

if sync.enabled == true then
    print("startup sync: " .. table.concat(sync.sources, " -> "))

    local ok, err = pcall(function()
        shell.run("sync", table.unpack(sync.sources))
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
print("startup: display controller")

local input_task = require("input_task")
local telemetry_task = require("telemetry_task")
local monitor_task = require("monitor_task")
local attitude_display = require("attitude_display")

local args = { ... }

local function monitorName(value)
    if value == nil or value == "" or value == "-" then
        return nil
    end

    return value
end

local shared = {
    displays = {
        main = monitorName(args[1] or config.displays.main),
        attitude = monitorName(args[2] or config.displays.attitude),
    },
    input = input_task.defaultInput(),
    inputTime = 0.0,
    inputSeq = 0,
    pendingNavigationCommand = nil,
    telemetry = nil,
    telemetryTime = 0.0,
    telemetrySender = nil,
    telemetryError = nil,
    running = true,
}

parallel.waitForAny(
    function()
        input_task.run(shared)
    end,
    function()
        telemetry_task.run(shared)
    end,
    function()
        monitor_task.run(shared)
    end,
    function()
        attitude_display.run(shared)
    end
)
