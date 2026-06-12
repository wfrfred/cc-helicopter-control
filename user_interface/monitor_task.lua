local display_alloc = require("display_alloc")
local monitor_view = require("monitor_view")
local config = require("config")

local monitor_task = {}

local TEXT_SCALE = config.monitor.text_scale
local DRAW_DT = config.monitor.draw_dt

local function waitForMonitor(shared)
    while shared.running do
        local mon = display_alloc.find(shared, "main")

        if mon then
            return mon
        end

        term.clear()
        term.setCursorPos(1, 1)
        print("main monitor not found")
        sleep(1)
    end
end

function monitor_task.run(shared)
    while shared.running do
        local mon = waitForMonitor(shared)

        if mon then
            mon.setTextScale(TEXT_SCALE)
            mon.setCursorBlink(false)

            while shared.running do
                local ok, err = pcall(monitor_view.draw, mon, shared)

                if not ok then
                    term.clear()
                    term.setCursorPos(1, 1)
                    print("monitor draw error:")
                    print(err)
                    sleep(1)
                    break
                end

                sleep(DRAW_DT)
            end
        end
    end
end

return monitor_task
