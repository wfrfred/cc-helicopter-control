local draw = require("draw")

local monitor_view = {}

local STALE_TELEMETRY_DT = 0.5

local function clamp(x, lo, hi)
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

local function deg(x)
    return math.deg(x)
end

local function fmt(value, pattern)
    return (pattern or "%.2f"):format(value)
end

local function cell(value, pattern, width)
    return draw.clip(fmt(value, pattern), width)
end

local function expectTable(value, name)
    assert(type(value) == "table", name .. " must be table")
    return value
end

local function section(mon, y, title, fg, bg)
    local w = mon.getSize()

    if y < 1 then
        return
    end

    draw.writeAt(mon, 1, y, string.upper(title), fg, bg, w)
end

local function drawOutput(mon, x, y, width, label, value, limit)
    if width < 16 then
        draw.writeAt(mon, x, y, ("%s %.1f"):format(label, value), colors.white, colors.black, width)
        return
    end

    draw.writeAt(mon, x, y, label, colors.lightGray, colors.black, 5)

    local bx = x + 6
    local bw = width - 13
    local pct = math.abs(clamp(value / limit, -1.0, 1.0))
    local len = math.floor(bw * pct + 0.5)
    local bg = value >= 0 and colors.blue or colors.purple

    draw.fill(mon, bx, y, bw, colors.gray)
    draw.fill(mon, bx, y, len, bg)
    draw.writeAt(mon, bx + bw + 1, y, ("%+.1f"):format(value), colors.white, colors.black, 7)
end

local function drawBladeOutput(mon, x, y, width, label, value)
    drawOutput(mon, x, y, width, label, value, 15.0)
end

local function drawRotorOutputs(mon, x, y, width, limitY, output)
    local rotor = expectTable(output.rotor, "telemetry.output.rotor")
    local upper = expectTable(rotor.upper, "telemetry.output.rotor.upper")
    local lower = expectTable(rotor.lower, "telemetry.output.rotor.lower")
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

    if width >= 62 then
        text = ("%-5s %7s %7s %7s %7s %7s %7s"):format("AXIS", "TARGET", "CURRENT", "ERROR", "P", "I", "D")
    elseif width >= 48 then
        text = ("%-5s %6s %6s %6s %6s %6s %6s"):format("AXIS", "TGT", "CUR", "ERR", "P", "I", "D")
    elseif width >= 36 then
        text = ("%-4s %5s %5s %5s  %s"):format("AX", "TGT", "CUR", "ERR", "P/I/D")
    else
        text = "AX TGT CUR ERR P/I/D"
    end

    draw.writeAt(mon, x, y, text, colors.lightGray, colors.black, width)
end

local function drawControllerRow(mon, x, y, width, label, target, current, err, angle, terms)
    expectTable(terms, label .. " pid terms")

    if angle then
        target = deg(target)
        current = deg(current)
        err = deg(err)
    end

    local text

    if width >= 62 then
        text = ("%-5s %7s %7s %7s %7s %7s %7s"):format(
            label,
            cell(target, "%.1f", 7),
            cell(current, "%.1f", 7),
            cell(err, "%.1f", 7),
            cell(terms.p, "%+.1f", 7),
            cell(terms.i, "%+.1f", 7),
            cell(terms.d, "%+.1f", 7)
        )
    elseif width >= 48 then
        text = ("%-5s %6s %6s %6s %6s %6s %6s"):format(
            label,
            cell(target, "%.1f", 6),
            cell(current, "%.1f", 6),
            cell(err, "%.1f", 6),
            cell(terms.p, "%+.1f", 6),
            cell(terms.i, "%+.1f", 6),
            cell(terms.d, "%+.1f", 6)
        )
    elseif width >= 36 then
        text = ("%-4s %5s %5s %5s  %s/%s/%s"):format(
            label,
            cell(target, "%.1f", 5),
            cell(current, "%.1f", 5),
            cell(err, "%.1f", 5),
            cell(terms.p, "%+.0f", 3),
            cell(terms.i, "%+.0f", 3),
            cell(terms.d, "%+.0f", 3)
        )
    else
        text = ("%s %s/%s/%s %s/%s/%s"):format(
            label,
            fmt(target, "%.1f"),
            fmt(current, "%.1f"),
            fmt(err, "%.1f"),
            fmt(terms.p, "%+.0f"),
            fmt(terms.i, "%+.0f"),
            fmt(terms.d, "%+.0f")
        )
    end

    draw.writeAt(mon, x, y, text, colors.white, colors.black, width)
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

        draw.writeAt(mon, x, y, table.concat(parts, "  "), fg, colors.black, width)
        y = y + 1
    end

    return y
end

local function drawFlightState(mon, x, y, width, limitY, telemetry)
    if y > limitY then
        return y
    end

    local current = expectTable(telemetry.current, "telemetry.current")
    local position = expectTable(telemetry.position, "telemetry.position")
    local velocity = expectTable(current.velocity, "telemetry.current.velocity")
    local items = {
        { label = "ALT", value = current.height, pattern = "%.1f" },
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

    return drawMetricGroups(mon, x, y, width, limitY, items, colors.white)
end

local function drawWaiting(mon, shared)
    local w, h = mon.getSize()
    draw.clear(mon, colors.black)
    draw.writeAt(mon, 1, 1, " HELI INPUT / DISPLAY", colors.black, colors.lime, w)
    draw.writeAt(mon, 1, 3, "waiting for telemetry", colors.white, colors.black, w)
    draw.writeAt(mon, 1, h, ("input seq %d"):format(shared.inputSeq), colors.lightGray, colors.gray, w)
end

local function drawNonRunning(mon, shared, telemetry)
    local w, h = mon.getSize()
    draw.clear(mon, colors.black)
    draw.writeAt(mon, 1, 1, " HELI INPUT / DISPLAY", colors.black, colors.lime, w)
    draw.writeAt(mon, 1, 3, "status " .. telemetry.status, colors.black, colors.orange, w)

    if telemetry.status == "waiting_sensors" then
        draw.writeAt(mon, 1, 5, "pose      " .. tostring(telemetry.haveState), colors.white, colors.black, w)
        draw.writeAt(mon, 1, 6, "yaw rate  " .. tostring(telemetry.haveYawRate), colors.white, colors.black, w)
        draw.writeAt(mon, 1, 7, "velocity  " .. tostring(telemetry.haveVelocity), colors.white, colors.black, w)
    end

    draw.writeAt(mon, 1, h, ("input seq %d"):format(shared.inputSeq), colors.lightGray, colors.gray, w)
end

local function drawRunning(mon, shared, telemetry)
    local w, h = mon.getSize()
    local target = expectTable(telemetry.target, "telemetry.target")
    local current = expectTable(telemetry.current, "telemetry.current")
    local err = expectTable(telemetry.error, "telemetry.error")
    local output = expectTable(telemetry.output, "telemetry.output")
    local pidData = expectTable(telemetry.pid, "telemetry.pid")
    local now = os.clock()

    assert(shared.telemetryTime > 0, "shared.telemetryTime must be set")

    local telemetryAge = now - shared.telemetryTime
    local inputAge = now - shared.inputTime
    local staleTelemetry = telemetryAge > STALE_TELEMETRY_DT

    draw.clear(mon, colors.black)

    draw.writeAt(mon, 1, 1, " HELI INPUT / DISPLAY", colors.black, colors.lime, w)

    local stateColor = colors.green
    if staleTelemetry or telemetry.inputStale then
        stateColor = colors.orange
    end

    draw.writeAt(mon, 1, 2, " running", colors.black, stateColor, math.min(w, 22))
    draw.writeAt(mon, 24, 2, ("ctl %.2fs  in %.2fs"):format(
        telemetryAge,
        inputAge
    ), colors.lightGray, colors.black, math.max(0, w - 23))

    local y = 4

    if y <= h then
        section(mon, y, "controller", colors.black, colors.lightBlue)
        y = y + 1
    end
    if y <= h then drawControllerHeader(mon, 2, y, w - 2) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "ALT", target.height, current.height, err.height, false, pidData.height) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "VSPD", target.verticalSpeed, current.verticalSpeed, err.verticalSpeed, false, pidData.verticalSpeed) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "ROL", target.roll, current.roll, err.roll, true, pidData.roll) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "PIT", target.pitch, current.pitch, err.pitch, true, pidData.pitch) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "YAW", target.yaw, current.yaw, err.yaw, true, pidData.yawAngle) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "YRAT", target.yawRate, current.yawRate, err.yawRate, true, pidData.yawRate) y = y + 2 end

    if y <= h then
        section(mon, y, "controller outputs", colors.black, colors.yellow)
        y = y + 1
    end
    if y <= h then drawOutput(mon, 2, y, w - 2, "COL", output.collective, 10.0) y = y + 1 end
    if y <= h then drawOutput(mon, 2, y, w - 2, "CFF", output.collectiveFeedforward, 10.0) y = y + 1 end
    if y <= h then drawOutput(mon, 2, y, w - 2, "CFB", output.collectiveFeedback, 6.0) y = y + 1 end
    if y <= h then drawOutput(mon, 2, y, w - 2, "ROL", output.roll, 8.0) y = y + 1 end
    if y <= h then drawOutput(mon, 2, y, w - 2, "PIT", output.pitch, 12.0) y = y + 1 end
    if y <= h then drawOutput(mon, 2, y, w - 2, "YAW", output.yaw, 8.0) y = y + 1 end

    if y <= h then
        y = y + 1
        y = drawRotorOutputs(mon, 2, y, w - 2, h - 2, output)
    end

    if y <= h - 2 then
        y = y + 1
        y = drawFlightState(mon, 2, y, w - 2, h - 2, telemetry)
    end

    if h >= 3 then
        local footer = ("input seq %d  telemetry %s"):format(
            shared.inputSeq,
            tostring(shared.telemetrySender)
        )
        if staleTelemetry then
            footer = footer .. " STALE"
        end
        draw.writeAt(mon, 1, h - 1, footer, colors.lightGray, colors.black, w)

        draw.writeAt(mon, 1, h, "network monitor online", colors.white, colors.gray, w)
    end
end

function monitor_view.draw(mon, shared)
    local telemetry = shared.telemetry

    if telemetry == nil then
        drawWaiting(mon, shared)
        return
    end

    expectTable(telemetry, "shared.telemetry")

    if telemetry.status ~= "running" then
        drawNonRunning(mon, shared, telemetry)
        return
    end

    drawRunning(mon, shared, telemetry)
end

return monitor_view
