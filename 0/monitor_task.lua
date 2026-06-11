local display_alloc = require("display_alloc")
local config = require("config")

local monitor_task = {}

local TEXT_SCALE = config.monitor.text_scale
local DRAW_DT = config.monitor.draw_dt

local function clamp(x, lo, hi)
    x = tonumber(x) or 0
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

local function deg(x)
    if x == nil then
        return nil
    end

    return math.deg(x)
end

local function fmt(value, pattern, fallback)
    if value == nil then
        return fallback or "--"
    end

    return (pattern or "%.2f"):format(value)
end

local function clip(text, width)
    text = tostring(text or "")
    width = math.max(0, tonumber(width) or 0)

    if #text > width then
        return text:sub(1, width)
    end

    return text
end

local function cell(value, pattern, width)
    return clip(fmt(value, pattern), width)
end

local function setFg(mon, color)
    if mon.setTextColor then
        mon.setTextColor(color)
    end
end

local function setBg(mon, color)
    if mon.setBackgroundColor then
        mon.setBackgroundColor(color)
    end
end

local function writeAt(mon, x, y, text, fg, bg, width)
    local w, h = mon.getSize()

    if y < 1 or y > h or x > w then
        return
    end

    width = width or (w - x + 1)
    width = math.min(width, w - x + 1)

    if width <= 0 then
        return
    end

    local out = clip(text, width)

    setFg(mon, fg or colors.white)
    setBg(mon, bg or colors.black)
    mon.setCursorPos(x, y)
    mon.write(out .. string.rep(" ", math.max(0, width - #out)))
end

local function fill(mon, x, y, width, bg)
    if width <= 0 then
        return
    end

    setBg(mon, bg)
    mon.setCursorPos(x, y)
    mon.write(string.rep(" ", width))
end

local function clear(mon, bg)
    setBg(mon, bg or colors.black)
    mon.clear()
end

local function section(mon, y, title, fg, bg)
    local w = mon.getSize()

    if y < 1 then
        return
    end

    writeAt(mon, 1, y, string.upper(title), fg or colors.white, bg or colors.gray, w)
end

local function drawAxis(mon, x, y, width, label, value)
    value = clamp(value, -1.0, 1.0)

    if width < 14 then
        writeAt(mon, x, y, ("%s %+.1f"):format(label, value), colors.white, colors.black, width)
        return
    end

    writeAt(mon, x, y, label, colors.lightGray, colors.black, 6)

    local bx = x + 7
    local bw = width - 13
    local mid = math.floor((bw + 1) / 2)
    local active = value >= 0 and colors.green or colors.orange

    fill(mon, bx, y, bw, colors.gray)
    fill(mon, bx + mid - 1, y, 1, colors.white)

    if value > 0 then
        local len = math.floor((bw - mid) * value + 0.5)
        fill(mon, bx + mid, y, len, active)
    elseif value < 0 then
        local len = math.floor((mid - 1) * -value + 0.5)
        fill(mon, bx + mid - len - 1, y, len, active)
    end

    writeAt(mon, bx + bw + 1, y, ("%+.1f"):format(value), colors.white, colors.black, 6)
end

local function drawOutput(mon, x, y, width, label, value, limit)
    value = tonumber(value) or 0.0
    limit = math.max(1.0, tonumber(limit) or 1.0)

    if width < 16 then
        writeAt(mon, x, y, ("%s %.1f"):format(label, value), colors.white, colors.black, width)
        return
    end

    writeAt(mon, x, y, label, colors.lightGray, colors.black, 5)

    local bx = x + 6
    local bw = width - 13
    local pct = math.abs(clamp(value / limit, -1.0, 1.0))
    local len = math.floor(bw * pct + 0.5)
    local bg = value >= 0 and colors.blue or colors.purple

    fill(mon, bx, y, bw, colors.gray)
    fill(mon, bx, y, len, bg)
    writeAt(mon, bx + bw + 1, y, ("%+.1f"):format(value), colors.white, colors.black, 7)
end

local function drawMiniSigned(mon, x, y, width, value, limit, color)
    value = tonumber(value) or 0.0
    limit = math.max(0.000001, tonumber(limit) or 1.0)

    local mid = math.floor((width + 1) / 2)
    local pct = math.abs(clamp(value / limit, -1.0, 1.0))

    fill(mon, x, y, width, colors.gray)
    fill(mon, x + mid - 1, y, 1, colors.white)

    if value > 0 then
        local len = math.floor((width - mid) * pct + 0.5)
        fill(mon, x + mid, y, len, color)
    elseif value < 0 then
        local len = math.floor((mid - 1) * pct + 0.5)
        fill(mon, x + mid - len - 1, y, len, color)
    end
end

local function drawPidTerms(mon, x, y, width, terms, limit)
    terms = terms or {}
    limit = math.max(0.000001, tonumber(limit) or 1.0)

    if width < 12 then
        local raw = tonumber(terms.raw) or 0.0
        local bg = math.abs(raw) > limit and colors.red or colors.gray
        writeAt(mon, x, y, "PID", colors.white, bg, width)
        return
    end

    local gap = 1
    local termWidth = math.floor((width - 2 * gap) / 3)
    local raw = tonumber(terms.raw) or 0.0
    local over = math.abs(raw) > limit

    drawMiniSigned(mon, x, y, termWidth, terms.p, limit, colors.blue)
    drawMiniSigned(mon, x + termWidth + gap, y, termWidth, terms.i, limit, colors.green)
    drawMiniSigned(mon, x + 2 * (termWidth + gap), y, termWidth, terms.d, limit, colors.orange)

    if over then
        writeAt(mon, x + width - 1, y, "!", colors.white, colors.red, 1)
    end
end

local function drawOutputWithPid(mon, x, y, width, label, value, limit, terms)
    if width < 46 or type(terms) ~= "table" then
        drawOutput(mon, x, y, width, label, value, limit)
        return
    end

    value = tonumber(value) or 0.0
    limit = math.max(1.0, tonumber(limit) or 1.0)

    writeAt(mon, x, y, label, colors.lightGray, colors.black, 5)

    local pidWidth = width >= 62 and 18 or 12
    local valueWidth = 7
    local bx = x + 6
    local bw = width - 6 - pidWidth - valueWidth - 2
    local pct = math.abs(clamp(value / limit, -1.0, 1.0))
    local len = math.floor(bw * pct + 0.5)
    local bg = value >= 0 and colors.blue or colors.purple
    local pidX = bx + bw + 1
    local valueX = pidX + pidWidth + 1

    fill(mon, bx, y, bw, colors.gray)
    fill(mon, bx, y, len, bg)
    drawPidTerms(mon, pidX, y, pidWidth, terms, limit)
    writeAt(mon, valueX, y, ("%+.1f"):format(value), colors.white, colors.black, valueWidth)
end

local function drawBladeOutput(mon, x, y, width, label, value)
    drawOutput(mon, x, y, width, label, value, 15.0)
end

local function drawRotorOutputs(mon, x, y, width, limitY, output)
    local rotor = output.rotor or {}
    local upper = rotor.upper or {}
    local lower = rotor.lower or {}
    local blades = {
        { label = "U-F", value = upper[1] },
        { label = "U-R", value = upper[2] },
        { label = "U-B", value = upper[3] },
        { label = "U-L", value = upper[4] },
        { label = "L-F", value = lower[1] },
        { label = "L-R", value = lower[2] },
        { label = "L-B", value = lower[3] },
        { label = "L-L", value = lower[4] },
    }

    if y > limitY then
        return y
    end

    section(mon, y, "blade outputs", colors.black, colors.orange)
    y = y + 1

    if width >= 58 then
        local colWidth = math.floor((width - 2) / 2)

        for i = 1, #blades, 2 do
            if y > limitY then
                return y
            end

            drawBladeOutput(mon, x, y, colWidth, blades[i].label, blades[i].value)
            drawBladeOutput(mon, x + colWidth + 2, y, colWidth, blades[i + 1].label, blades[i + 1].value)
            y = y + 1
        end
    else
        for _, blade in ipairs(blades) do
            if y > limitY then
                return y
            end

            drawBladeOutput(mon, x, y, width, blade.label, blade.value)
            y = y + 1
        end
    end

    return y
end

local function drawControllerHeader(mon, x, y, width)
    local text

    if width >= 28 then
        text = ("%-4s %7s %7s %7s"):format("AXIS", "TARGET", "CURRENT", "ERROR")
    elseif width >= 22 then
        text = ("%-3s %5s %5s %5s"):format("AX", "TGT", "CUR", "ERR")
    else
        text = "AX TGT CUR ERR"
    end

    writeAt(mon, x, y, text, colors.lightGray, colors.black, width)
end

local function drawPair(mon, x, y, width, label, target, current, err, angle)
    if angle then
        target = deg(target)
        current = deg(current)
        err = deg(err)
    end

    local text

    if width >= 28 then
        text = ("%-4s %7s %7s %7s"):format(
            label,
            cell(target, "%.1f", 7),
            cell(current, "%.1f", 7),
            cell(err, "%.1f", 7)
        )
    elseif width >= 22 then
        text = ("%-3s %5s %5s %5s"):format(
            label,
            cell(target, "%.0f", 5),
            cell(current, "%.0f", 5),
            cell(err, "%.0f", 5)
        )
    else
        text = ("%s %s/%s/%s"):format(
            label,
            fmt(target, "%.0f"),
            fmt(current, "%.0f"),
            fmt(err, "%.0f")
        )
    end

    writeAt(mon, x, y, text, colors.white, colors.black, width)
end

local function drawMetricGroups(mon, x, y, width, limitY, items, fg)
    local perLine = 1

    if width >= 60 then
        perLine = 4
    elseif width >= 44 then
        perLine = 3
    elseif width >= 28 then
        perLine = 2
    end

    local i = 1
    while i <= #items and y <= limitY do
        local parts = {}

        for _ = 1, perLine do
            local item = items[i]

            if not item then
                break
            end

            parts[#parts + 1] = ("%-5s %8s"):format(
                item.label,
                cell(item.value, item.pattern or "%.1f", 8)
            )
            i = i + 1
        end

        writeAt(mon, x, y, table.concat(parts, "  "), fg or colors.white, colors.black, width)
        y = y + 1
    end

    return y
end

local function drawFlightState(mon, x, y, width, limitY, telemetry)
    if y > limitY then
        return y
    end

    local current = telemetry.current or {}
    local position = telemetry.position or {}
    local velocity = current.velocity or {}
    local height = current.height or position.y
    local items = {
        { label = "ALT", value = height, pattern = "%.1f" },
        { label = "HSPD", value = velocity.horizontal, pattern = "%.1f" },
        { label = "VSPD", value = velocity.vertical, pattern = "%+.1f" },
        { label = "TSPD", value = velocity.total, pattern = "%.1f" },

        { label = "ROLL", value = deg(current.roll), pattern = "%+.1f" },
        { label = "PITCH", value = deg(current.pitch), pattern = "%+.1f" },
        { label = "YAW", value = deg(current.yaw), pattern = "%.1f" },
        { label = "YRATE", value = deg(current.yawRate), pattern = "%+.1f" },

        { label = "POSX", value = position.x, pattern = "%.1f" },
        { label = "POSY", value = position.y, pattern = "%.1f" },
        { label = "POSZ", value = position.z, pattern = "%.1f" },

        { label = "VELX", value = velocity.x, pattern = "%+.1f" },
        { label = "VELY", value = velocity.y, pattern = "%+.1f" },
        { label = "VELZ", value = velocity.z, pattern = "%+.1f" },
    }

    section(mon, y, "flight state", colors.black, colors.green)
    y = y + 1
    y = drawMetricGroups(mon, x, y, width, limitY, items, colors.white)

    return y
end

local function draw(mon, shared)
    local w, h = mon.getSize()
    local telemetry = shared.telemetry or {}
    local target = telemetry.target or {}
    local current = telemetry.current or {}
    local err = telemetry.error or {}
    local output = telemetry.output or {}
    local pidData = telemetry.pid or {}
    local now = os.clock()

    local telemetryAge
    if shared.telemetryTime and shared.telemetryTime > 0 then
        telemetryAge = now - shared.telemetryTime
    end

    local inputAge
    if shared.inputTime and shared.inputTime > 0 then
        inputAge = now - shared.inputTime
    end

    local status = telemetry.status or "waiting_telemetry"
    local staleTelemetry = telemetryAge == nil or telemetryAge > 0.5
    local inputStale = telemetry.inputStale

    clear(mon, colors.black)

    writeAt(mon, 1, 1, " HELI INPUT / DISPLAY", colors.black, colors.lime, w)

    local stateColor = colors.green
    if status ~= "running" or staleTelemetry or inputStale then
        stateColor = colors.orange
    end
    if telemetry.dataError or telemetry.inputError or shared.telemetryError then
        stateColor = colors.red
    end

    local colorText = ""
    if mon.isColor and mon.isColor() then
        colorText = " adv"
    end

    writeAt(mon, 1, 2, (" %s"):format(status), colors.black, stateColor, math.min(w, 22))
    writeAt(mon, 24, 2, ("ctl %s  in %s%s"):format(
        fmt(telemetryAge, "%.2fs", "--"),
        fmt(inputAge, "%.2fs", "--"),
        colorText
    ), colors.lightGray, colors.black, math.max(0, w - 23))

    local y = 4

    if y <= h then
        section(mon, y, "controller", colors.black, colors.lightBlue)
        y = y + 1
    end
    if y <= h then drawControllerHeader(mon, 2, y, w - 2) y = y + 1 end
    if y <= h then drawPair(mon, 2, y, w - 2, "ALT", target.height, current.height, err.height, false) y = y + 1 end
    if y <= h then drawPair(mon, 2, y, w - 2, "ROL", target.roll, current.roll, err.roll, true) y = y + 1 end
    if y <= h then drawPair(mon, 2, y, w - 2, "PIT", target.pitch, current.pitch, err.pitch, true) y = y + 1 end
    if y <= h then drawPair(mon, 2, y, w - 2, "YAW", target.yaw, current.yaw, err.yaw, true) y = y + 2 end

    if y <= h then
        section(mon, y, "controller outputs  pid p/i/d", colors.black, colors.yellow)
        y = y + 1
    end
    if y <= h then drawOutputWithPid(mon, 2, y, w - 2, "COL", output.collective, 10.0, pidData.height) y = y + 1 end
    if y <= h then drawOutputWithPid(mon, 2, y, w - 2, "ROL", output.roll, 8.0, pidData.roll) y = y + 1 end
    if y <= h then drawOutputWithPid(mon, 2, y, w - 2, "PIT", output.pitch, 12.0, pidData.pitch) y = y + 1 end
    if y <= h then drawOutputWithPid(mon, 2, y, w - 2, "YAW", output.yaw, 8.0, pidData.yawRate) y = y + 1 end
    if y <= h then drawOutputWithPid(mon, 2, y, w - 2, "YAW-A", deg(target.yawRate), 60.0, pidData.yawAngle) y = y + 1 end

    if y <= h then
        y = y + 1
        y = drawRotorOutputs(mon, 2, y, w - 2, h - 2, output)
    end

    if y <= h - 2 then
        y = y + 1
        y = drawFlightState(mon, 2, y, w - 2, h - 2, telemetry)
    end

    if h >= 3 then
        local footer = ("input seq %s  telemetry %s"):format(
            tostring(shared.inputSeq or 0),
            shared.telemetrySender and tostring(shared.telemetrySender) or "none"
        )
        if staleTelemetry then
            footer = footer .. " STALE"
        end
        writeAt(mon, 1, h - 1, footer, colors.lightGray, colors.black, w)

        local bottom = telemetry.dataError or telemetry.inputError or shared.telemetryError or "network monitor online"
        local bottomBg = (telemetry.dataError or telemetry.inputError or shared.telemetryError) and colors.red or colors.gray
        writeAt(mon, 1, h, bottom, colors.white, bottomBg, w)
    end
end

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
            pcall(function()
                if mon.setTextScale then
                    mon.setTextScale(TEXT_SCALE)
                end
                if mon.setCursorBlink then
                    mon.setCursorBlink(false)
                end
            end)

            while shared.running do
                local ok, err = pcall(draw, mon, shared)

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
