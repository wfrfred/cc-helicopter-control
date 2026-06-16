local display_alloc = require("display_alloc")
local monitor_view = require("monitor_view")
local config = require("config")

local monitor_task = {}

local TEXT_SCALE = config.monitor.text_scale
local DRAW_DT = config.monitor.draw_dt

local function waitForMonitor(shared)
    while shared.running do
        local mon, name = display_alloc.find(shared, "main")

        if mon then
            return mon, name
        end

        term.clear()
        term.setCursorPos(1, 1)
        print("main monitor not found")
        sleep(1)
    end
end

local function drawLoop(mon, shared)
    while shared.running do
        local ok, err = pcall(monitor_view.draw, mon, shared)

        if not ok then
            term.clear()
            term.setCursorPos(1, 1)
            print("monitor draw error:")
            print(err)
            sleep(1)
            return
        end

        sleep(DRAW_DT)
    end
end

local function touchLoop(mon, monitorName, shared)
    while shared.running do
        local event, side, x, y = os.pullEvent()

        if event == "monitor_touch" and (monitorName == nil or side == monitorName) then
            monitor_view.handleTouch(mon, shared, x, y)
        elseif event == "monitor_resize" and (monitorName == nil or side == monitorName) then
            sleep(0)
        end
    end
end

function monitor_task.run(shared)
    while shared.running do
        local mon, monitorName = waitForMonitor(shared)

        if mon then
            mon.setTextScale(TEXT_SCALE)
            mon.setCursorBlink(false)

            parallel.waitForAny(
                function()
                    drawLoop(mon, shared)
                end,
                function()
                    touchLoop(mon, monitorName, shared)
                end
            )
        end
    end
end

return monitor_task
