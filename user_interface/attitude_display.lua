local attitude_view = require("attitude_view")
local display_alloc = require("display_alloc")
local config = require("config")

local attitude_display = {}

local TEXT_SCALE = config.attitude.text_scale
local DRAW_DT = config.attitude.draw_dt

local function waitForMonitor(shared)
    while shared.running do
        local mon = display_alloc.find(shared, "attitude")

        if mon then
            return mon
        end

        term.clear()
        term.setCursorPos(1, 1)
        print("attitude monitor not found")
        sleep(1)
    end
end

function attitude_display.run(shared)
    while shared.running do
        local mon = waitForMonitor(shared)

        if mon then
            mon.setTextScale(TEXT_SCALE)
            mon.setCursorBlink(false)

            while shared.running do
                local ok, err = pcall(attitude_view.draw, mon, shared)

                if not ok then
                    term.clear()
                    term.setCursorPos(1, 1)
                    print("attitude draw error:")
                    print(err)
                    sleep(1)
                    break
                end

                sleep(DRAW_DT)
            end
        end
    end
end

return attitude_display
