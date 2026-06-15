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

local function velocityTotal(velocity)
    return math.sqrt(
        velocity.x * velocity.x
            + velocity.y * velocity.y
            + velocity.z * velocity.z
    )
end

local function velocityHorizontal(velocity)
    return math.sqrt(velocity.x * velocity.x + velocity.z * velocity.z)
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

local function drawOutputBar(mon, x, y, width, value, limit)
    if width < 9 then
        draw.writeAt(mon, x, y, ("%+.1f"):format(value), colors.white, colors.black, width)
        return
    end

    local valueWidth = 7
    local barWidth = width - valueWidth - 1
    local pct = math.abs(clamp(value / limit, -1.0, 1.0))
    local len = math.floor(barWidth * pct + 0.5)
    local bg = value >= 0 and colors.blue or colors.purple

    draw.fill(mon, x, y, barWidth, colors.gray)
    draw.fill(mon, x, y, len, bg)
    draw.writeAt(mon, x + barWidth + 1, y, ("%+.1f"):format(value), colors.white, colors.black, valueWidth)
end

local function drawCompactBar(mon, x, y, width, label, value, limit)
    local barWidth = width - 8
    local length = math.floor(barWidth * math.abs(clamp(value / limit, -1.0, 1.0)) + 0.5)
    local bg = value >= 0 and colors.blue or colors.purple

    draw.writeAt(mon, x, y, label, colors.lightGray, colors.black, 2)
    draw.fill(mon, x + 3, y, barWidth, colors.gray)
    draw.fill(mon, x + 3, y, length, bg)
    draw.writeAt(mon, x + width - 4, y, ("%+.1f"):format(value), colors.white, colors.black, 5)
end

local function drawBladeOutputRow(mon, x, y, width, blades)
    local columnWidth = math.floor((width - 3) / 4)

    for index, blade in ipairs(blades) do
        local columnX = x + (index - 1) * (columnWidth + 1)
        drawCompactBar(mon, columnX, y, columnWidth, blade.label, blade.value, 15.0)
    end
end

local function drawRotorOutputs(mon, x, y, width, limitY, output)
    local rotor = expectTable(output.rotor, "telemetry.output.rotor")
    local upper = expectTable(rotor.upper, "telemetry.output.rotor.upper")
    local lower = expectTable(rotor.lower, "telemetry.output.rotor.lower")
    local rows = {
        {
            { label = "UF", value = upper[1] },
            { label = "UR", value = upper[2] },
            { label = "UB", value = upper[3] },
            { label = "UL", value = upper[4] },
        },
        {
            { label = "LF", value = lower[1] },
            { label = "LR", value = lower[2] },
            { label = "LB", value = lower[3] },
            { label = "LL", value = lower[4] },
        },
    }

    if y > limitY then
        return y
    end

    section(mon, y, "blade outputs", colors.black, colors.orange)
    y = y + 1

    for _, row in ipairs(rows) do
        if y > limitY then
            return y
        end

        drawBladeOutputRow(mon, x, y, width, row)
        y = y + 1
    end

    return y
end

local function drawControllerOutputs(mon, x, y, width, limitY, output)
    if y > limitY then
        return y
    end

    section(mon, y, "controller outputs", colors.black, colors.yellow)
    y = y + 1

    local columnWidth = math.floor((width - 2) / 2)
    local commands = expectTable(output.commands, "telemetry.output.commands")
    local collective = expectTable(output.collective, "telemetry.output.collective")
    local rows = {
        {
            { label = "COL", value = commands.collective },
            { label = "ROL", value = commands.roll },
        },
        {
            { label = "CFF", value = collective.feedforward },
            { label = "PIT", value = commands.pitch },
        },
        {
            { label = "CFB", value = collective.feedback },
            { label = "YAW", value = commands.yaw },
        },
    }
    local limits = {
        { 10.0, 8.0 },
        { 10.0, 12.0 },
        { 6.0, 8.0 },
    }

    for index, row in ipairs(rows) do
        if y > limitY then
            return y
        end

        drawOutput(mon, x, y, columnWidth, row[1].label, row[1].value, limits[index][1])
        drawOutput(mon, x + columnWidth + 2, y, columnWidth, row[2].label, row[2].value, limits[index][2])
        y = y + 1
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
    local state = expectTable(telemetry.state, "telemetry.state")
    local raw = expectTable(state.raw, "telemetry.state.raw")
    local body = expectTable(state.body, "telemetry.state.body")
    local position = expectTable(raw.position, "telemetry.state.raw.position")
    local velocity = expectTable(raw.velocity, "telemetry.state.raw.velocity")
    local attitude = expectTable(current.attitude, "telemetry.current.attitude")
    local yaw = expectTable(current.yaw, "telemetry.current.yaw")
    local vertical = expectTable(current.vertical, "telemetry.current.vertical")
    local bodyVelocity = expectTable(body.velocity, "telemetry.state.body.velocity")
    local items = {
        { label = "ALT", value = vertical.height, pattern = "%.1f" },
        { label = "HSPD", value = velocityHorizontal(velocity), pattern = "%.1f" },
        { label = "VSPD", value = vertical.speed, pattern = "%+.1f" },
        { label = "TSPD", value = velocityTotal(velocity), pattern = "%.1f" },

        { label = "ROLL", value = deg(attitude.roll), pattern = "%+.1f" },
        { label = "PITCH", value = deg(attitude.pitch), pattern = "%+.1f" },
        { label = "YAW", value = deg(yaw.angle), pattern = "%.1f" },
        { label = "YRATE", value = deg(yaw.rate), pattern = "%+.1f" },

        { label = "POSX", value = position.x, pattern = "%.1f" },
        { label = "POSY", value = position.y, pattern = "%.1f" },
        { label = "POSZ", value = position.z, pattern = "%.1f" },

        { label = "VELX", value = velocity.x, pattern = "%+.1f" },
        { label = "VELY", value = velocity.y, pattern = "%+.1f" },
        { label = "VELZ", value = velocity.z, pattern = "%+.1f" },
        { label = "BFWD", value = bodyVelocity.forward, pattern = "%+.1f" },
        { label = "BRGT", value = bodyVelocity.right, pattern = "%+.1f" },
        { label = "BDWN", value = bodyVelocity.down, pattern = "%+.1f" },
    }

    section(mon, y, "flight state", colors.black, colors.green)
    y = y + 1

    return drawMetricGroups(mon, x, y, width, limitY, items, colors.white)
end

local function scaledOffset(value, limit, radius)
    local scaled = clamp(value / limit, -1.0, 1.0)
    return math.floor(scaled * radius + (scaled >= 0 and 0.5 or -0.5))
end

local function drawAxisBar(mon, x, y, width, label, value, limit)
    if width < 10 then
        draw.writeAt(mon, x, y, ("%s %+.1f"):format(label, value), colors.white, colors.black, width)
        return
    end

    local barWidth = width - 8
    local center = x + 5 + math.floor((barWidth - 1) / 2)
    local mark = center + scaledOffset(value, limit, math.floor((barWidth - 1) / 2))

    draw.writeAt(mon, x, y, label, colors.lightGray, colors.black, 4)
    draw.fill(mon, x + 5, y, barWidth, colors.gray)
    draw.writeAt(mon, center, y, "+", colors.black, colors.yellow, 1)
    draw.writeAt(mon, mark, y, "*", colors.white, colors.red, 1)
    draw.writeAt(mon, x + width - 3, y, ("%+.1f"):format(value), colors.white, colors.black, 4)
end

local function drawPositionMap(mon, x, y, width, height, target, current)
    if height < 1 then
        return
    end

    if height > 2 and height % 2 == 0 then
        height = height - 1
    end

    if width < 12 or height < 3 then
        drawAxisBar(mon, x, y, width, "R", current.right - target.right, 10.0)
        if height >= 2 then
            drawAxisBar(mon, x, y + 1, width, "F", current.forward - target.forward, 10.0)
        end
        return
    end

    local centerX = x + math.floor(width / 2)
    local centerY = y + math.floor(height / 2)
    local markX = centerX + scaledOffset(current.right - target.right, 10.0, math.floor((width - 3) / 2))
    local markY = centerY - scaledOffset(current.forward - target.forward, 10.0, math.floor((height - 1) / 2))

    for row = 0, height - 1 do
        local lineY = y + row
        draw.writeAt(mon, x, lineY, string.rep(" ", width), colors.white, colors.black, width)
        draw.writeAt(mon, centerX, lineY, "|", colors.gray, colors.black, 1)
    end

    draw.writeAt(mon, x, centerY, string.rep("-", width), colors.gray, colors.black, width)
    draw.writeAt(mon, centerX, centerY, "+", colors.black, colors.yellow, 1)
    draw.writeAt(mon, markX, markY, "C", colors.white, colors.red, 1)
end

local function drawPositionColumn(mon, x, y, width, height, target, current)
    if height < 1 then
        return
    end

    drawPositionMap(mon, x, y, width, height, target, current)
end

local function drawPositionPidColumn(mon, x, y, width, height, rows)
    if height < 1 then
        return
    end

    local header

    if width >= 32 then
        header = ("%-5s %7s %7s %7s"):format("AXIS", "TGT", "CUR", "ERR")
    elseif width >= 24 then
        header = ("%-4s %6s %6s %6s"):format("AX", "TGT", "CUR", "ERR")
    else
        header = "AX TGT/CUR/ERR"
    end

    draw.writeAt(mon, x, y, header, colors.lightGray, colors.black, width)

    for index, row in ipairs(rows) do
        local rowY = y + index

        if rowY >= y + height then
            return
        end

        local text

        if width >= 32 then
            text = ("%-5s %7s %7s %7s"):format(
                row.label,
                cell(row.target, "%.1f", 7),
                cell(row.current, "%.1f", 7),
                cell(row.err, "%+.1f", 7)
            )
        elseif width >= 24 then
            text = ("%-4s %6s %6s %6s"):format(
                row.label,
                cell(row.target, "%.1f", 6),
                cell(row.current, "%.1f", 6),
                cell(row.err, "%+.1f", 6)
            )
        else
            text = ("%s %s/%s/%s"):format(
                row.label,
                cell(row.target, "%.1f", 5),
                cell(row.current, "%.1f", 5),
                cell(row.err, "%+.1f", 5)
            )
        end

        draw.writeAt(mon, x, rowY, text, colors.white, colors.black, width)
    end
end

local function drawPositionPidOutputColumn(mon, x, y, width, height, rows)
    if height < 1 then
        return
    end

    draw.writeAt(mon, x, y, "PID OUTPUT", colors.lightGray, colors.black, width)

    for index, row in ipairs(rows) do
        local rowY = y + index

        if rowY >= y + height then
            return
        end

        drawOutputBar(mon, x, rowY, width, row.value, row.limit)
    end
end

local function drawPositionHold(mon, x, y, width, limitY, telemetry)
    if y > limitY then
        return y
    end

    local positionHold = expectTable(telemetry.positionHold, "telemetry.positionHold")
    local pidData = expectTable(telemetry.pid, "telemetry.pid")
    local position = expectTable(positionHold.position, "telemetry.positionHold.position")
    local velocity = expectTable(positionHold.velocity, "telemetry.positionHold.velocity")
    local output = expectTable(positionHold.output, "telemetry.positionHold.output")
    local target = expectTable(position.target, "telemetry.positionHold.position.target")
    local currentPosition = expectTable(position.current, "telemetry.positionHold.position.current")
    local targetVelocity = expectTable(velocity.target, "telemetry.positionHold.velocity.target")
    local currentVelocity = expectTable(velocity.current, "telemetry.positionHold.velocity.current")
    local err = expectTable(position.error, "telemetry.positionHold.position.error")
    local positionPid = expectTable(pidData.position, "telemetry.pid.position")
    local velocityPid = expectTable(pidData.velocity, "telemetry.pid.velocity")
    local positionRightTerms = expectTable(positionPid.right, "telemetry.pid.position.right")
    local positionForwardTerms = expectTable(positionPid.forward, "telemetry.pid.position.forward")
    local velocityRightTerms = expectTable(velocityPid.right, "telemetry.pid.velocity.right")
    local velocityForwardTerms = expectTable(velocityPid.forward, "telemetry.pid.velocity.forward")

    section(mon, y, "position hold", colors.black, colors.pink)
    y = y + 1

    if not positionHold.active then
        if y <= limitY then
            draw.writeAt(mon, x, y, "manual roll/pitch", colors.lightGray, colors.black, width)
            y = y + 1
        end
        return y
    end

    local gap = 2
    local contentWidth = width - gap * 2
    local positionWidth = math.floor(contentWidth * 0.30)
    local outputWidth = math.max(12, math.floor(contentWidth * 0.24))
    local pidWidth = contentWidth - positionWidth - outputWidth

    if width < 58 then
        gap = 1
        contentWidth = width - gap * 2
        positionWidth = math.floor(contentWidth * 0.30)
        outputWidth = math.max(10, math.floor(contentWidth * 0.24))
        pidWidth = contentWidth - positionWidth - outputWidth
    end

    if positionWidth < 10 or pidWidth < 14 or outputWidth < 8 then
        drawAxisBar(mon, x, y, width, "R", err.right, 10.0)
        if y + 1 <= limitY then
            drawAxisBar(mon, x, y + 1, width, "F", err.forward, 10.0)
            return y + 2
        end
        return y + 1
    end

    local rows = {
        {
            label = "RPOS",
            target = 0.0,
            current = -err.right,
            err = err.right,
        },
        {
            label = "FPOS",
            target = 0.0,
            current = -err.forward,
            err = err.forward,
        },
        {
            label = "RVEL",
            target = targetVelocity.right,
            current = currentVelocity.right,
            err = targetVelocity.right - currentVelocity.right,
        },
        {
            label = "FVEL",
            target = targetVelocity.forward,
            current = currentVelocity.forward,
            err = targetVelocity.forward - currentVelocity.forward,
        },
    }
    local outputRows = {
        { value = output.right.value, limit = 20.0 },
        { value = output.forward.value, limit = 20.0 },
        { value = deg(velocityRightTerms.output), limit = 20.0 },
        { value = deg(velocityForwardTerms.output), limit = 30.0 },
    }
    local bodyHeight = math.min(5, limitY - y + 1)
    local positionColumnX = x
    local pidX = positionColumnX + positionWidth + gap
    local outputColumnX = pidX + pidWidth + gap

    drawPositionColumn(mon, positionColumnX, y, positionWidth, bodyHeight, target, currentPosition)
    drawPositionPidColumn(mon, pidX, y, pidWidth, bodyHeight, rows)
    drawPositionPidOutputColumn(mon, outputColumnX, y, outputWidth, bodyHeight, outputRows)

    y = y + bodyHeight

    return y
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
        draw.writeAt(mon, 1, 5, "pose      " .. tostring(telemetry.havePose), colors.white, colors.black, w)
        draw.writeAt(mon, 1, 6, "rates     " .. tostring(telemetry.haveRates), colors.white, colors.black, w)
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
    local verticalPid = expectTable(pidData.vertical, "telemetry.pid.vertical")
    local attitudePid = expectTable(pidData.attitude, "telemetry.pid.attitude")
    local yawPid = expectTable(pidData.yaw, "telemetry.pid.yaw")
    local targetVertical = expectTable(target.vertical, "telemetry.target.vertical")
    local targetAttitude = expectTable(target.attitude, "telemetry.target.attitude")
    local targetAttitudeRate = expectTable(targetAttitude.rate, "telemetry.target.attitude.rate")
    local targetYaw = expectTable(target.yaw, "telemetry.target.yaw")
    local currentVertical = expectTable(current.vertical, "telemetry.current.vertical")
    local currentAttitude = expectTable(current.attitude, "telemetry.current.attitude")
    local currentAttitudeRate = expectTable(currentAttitude.rate, "telemetry.current.attitude.rate")
    local currentYaw = expectTable(current.yaw, "telemetry.current.yaw")
    local errorVertical = expectTable(err.vertical, "telemetry.error.vertical")
    local errorAttitude = expectTable(err.attitude, "telemetry.error.attitude")
    local errorAttitudeRate = expectTable(errorAttitude.rate, "telemetry.error.attitude.rate")
    local errorYaw = expectTable(err.yaw, "telemetry.error.yaw")
    local inputTelemetry = expectTable(telemetry.input, "telemetry.input")
    local now = os.clock()

    assert(shared.telemetryTime > 0, "shared.telemetryTime must be set")

    local telemetryAge = now - shared.telemetryTime
    local inputAge = inputTelemetry.age
    local staleTelemetry = telemetryAge > STALE_TELEMETRY_DT

    draw.clear(mon, colors.black)

    draw.writeAt(mon, 1, 1, " HELI INPUT / DISPLAY", colors.black, colors.lime, w)

    local stateColor = colors.green
    if staleTelemetry or inputTelemetry.stale then
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
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "ALT", targetVertical.height, currentVertical.height, errorVertical.height, false, verticalPid.height) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "VSPD", targetVertical.speed, currentVertical.speed, errorVertical.speed, false, verticalPid.speed) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "ROL", targetAttitude.roll, currentAttitude.roll, errorAttitude.roll, true, attitudePid.roll.angle) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "RRAT", targetAttitudeRate.roll, currentAttitudeRate.roll, errorAttitudeRate.roll, true, attitudePid.roll.rate) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "PIT", targetAttitude.pitch, currentAttitude.pitch, errorAttitude.pitch, true, attitudePid.pitch.angle) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "PRAT", targetAttitudeRate.pitch, currentAttitudeRate.pitch, errorAttitudeRate.pitch, true, attitudePid.pitch.rate) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "YAW", targetYaw.angle, currentYaw.angle, errorYaw.angle, true, yawPid.angle) y = y + 1 end
    if y <= h then drawControllerRow(mon, 2, y, w - 2, "YRAT", targetYaw.rate, currentYaw.rate, errorYaw.rate, true, yawPid.rate) y = y + 2 end

    y = drawControllerOutputs(mon, 2, y, w - 2, h, output)

    if y <= h then
        y = y + 1
        y = drawRotorOutputs(mon, 2, y, w - 2, h - 2, output)
    end

    if y <= h - 2 then
        y = y + 1
        y = drawPositionHold(mon, 2, y, w - 2, h - 2, telemetry)
    end

    if y <= h - 2 then
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
