local draw = require("draw")

local monitor_view = {}

local STALE_TELEMETRY_DT = 0.5
local PANEL_GAP = 2
local STATE_COLUMN_WIDTH = 26
local OUTPUT_COLUMN_WIDTH = 24
local POSITION_STATE_COLUMN_WIDTH = 18
local POSITION_OUTPUT_COLUMN_WIDTH = 14
local MIN_STATE_COLUMN_WIDTH = 14
local MIN_OUTPUT_COLUMN_WIDTH = 12

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

local function leftText(text, width)
    return draw.clip(text, width)
end

local function rightText(text, width)
    local clipped = draw.clip(text, width)
    return string.rep(" ", width - #clipped) .. clipped
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

local function positionHoldLayout(x, width)
    local gap = PANEL_GAP
    local stateWidth = math.min(
        POSITION_STATE_COLUMN_WIDTH,
        math.max(MIN_STATE_COLUMN_WIDTH, math.floor(width * 0.18))
    )
    local outputWidth = math.min(
        POSITION_OUTPUT_COLUMN_WIDTH,
        math.max(MIN_OUTPUT_COLUMN_WIDTH, math.floor(width * 0.14))
    )
    local pidWidth = width - stateWidth - outputWidth - gap * 2

    if pidWidth < 48 then
        local need = 48 - pidWidth
        local stateReduce = math.min(need, stateWidth - MIN_STATE_COLUMN_WIDTH)
        stateWidth = stateWidth - stateReduce
        need = need - stateReduce

        local outputReduce = math.min(need, outputWidth - MIN_OUTPUT_COLUMN_WIDTH)
        outputWidth = outputWidth - outputReduce
        pidWidth = width - stateWidth - outputWidth - gap * 2
    end

    return {
        stateX = x,
        stateWidth = stateWidth,
        pidX = x + stateWidth + gap,
        pidWidth = math.max(1, pidWidth),
        outputX = x + stateWidth + gap + math.max(1, pidWidth) + gap,
        outputWidth = outputWidth,
    }
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
    if width < 8 then
        draw.writeAt(mon, x, y, ("%+.1f"):format(value), colors.white, colors.black, width)
        return
    end

    local valueWidth = math.min(6, math.max(4, width - 2))
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

local function pidTableSpec(width)
    if width >= 62 then
        return {
            labels = { "AXIS", "TARGET", "CURRENT", "ERROR", "P", "I", "D" },
            widths = { 5, 7, 7, 7, 7, 7, 7 },
            valuePattern = "%.1f",
            termPattern = "%+.1f",
        }
    end

    if width >= 48 then
        return {
            labels = { "AXIS", "TGT", "CUR", "ERR", "P", "I", "D" },
            widths = { 5, 6, 6, 6, 6, 6, 6 },
            valuePattern = "%.1f",
            termPattern = "%+.1f",
        }
    end

    if width >= 36 then
        return {
            labels = { "AX", "TGT", "CUR", "ERR", "P", "I", "D" },
            widths = { 4, 5, 5, 5, 4, 4, 4 },
            valuePattern = "%.1f",
            termPattern = "%+.1f",
        }
    end

    return {
        labels = { "AX", "TGT", "CUR", "ERR", "P", "I", "D" },
        widths = { 3, 4, 4, 4, 4, 4, 4 },
        valuePattern = "%.1f",
        termPattern = "%+.1f",
    }
end

local function pidTableLayout(width, columnWidths)
    local gapCount = #columnWidths - 1
    local totalColumnWidth = 0

    for _, columnWidth in ipairs(columnWidths) do
        totalColumnWidth = totalColumnWidth + columnWidth
    end

    local minGap = width >= totalColumnWidth + gapCount and 1 or 0
    local spare = math.max(0, width - totalColumnWidth - minGap * gapCount)
    local baseExtra = gapCount > 0 and math.floor(spare / gapCount) or 0
    local remainder = gapCount > 0 and (spare - baseExtra * gapCount) or 0
    local layout = {}
    local cursor = 0

    for index, columnWidth in ipairs(columnWidths) do
        layout[index] = {
            offset = cursor,
            width = columnWidth,
        }

        if index < #columnWidths then
            local gap = minGap + baseExtra
            if index <= remainder then
                gap = gap + 1
            end
            cursor = cursor + columnWidth + gap
        end
    end

    return layout
end

local function drawPidTableCells(mon, x, y, width, values, fg)
    local spec = pidTableSpec(width)
    local layout = pidTableLayout(width, spec.widths)

    for index, value in ipairs(values) do
        local column = layout[index]
        local text = index == 1 and leftText(value, column.width) or rightText(value, column.width)

        draw.writeAt(mon, x + column.offset, y, text, fg, colors.black, column.width)
    end
end

local function drawControllerHeader(mon, x, y, width)
    drawPidTableCells(mon, x, y, width, pidTableSpec(width).labels, colors.lightGray)
end

local function drawControllerRow(mon, x, y, width, label, target, current, err, angle, terms, angularTerms)
    expectTable(terms, label .. " pid terms")

    if angle then
        target = deg(target)
        current = deg(current)
        err = deg(err)
    end

    local p = terms.p
    local i = terms.i
    local d = terms.d

    if angularTerms then
        p = deg(p)
        i = deg(i)
        d = deg(d)
    end

    local spec = pidTableSpec(width)
    drawPidTableCells(mon, x, y, width, {
        label,
        cell(target, spec.valuePattern, spec.widths[2]),
        cell(current, spec.valuePattern, spec.widths[3]),
        cell(err, spec.valuePattern, spec.widths[4]),
        cell(p, spec.termPattern, spec.widths[5]),
        cell(i, spec.termPattern, spec.widths[6]),
        cell(d, spec.termPattern, spec.widths[7]),
    }, colors.white)
end

local function drawValueRow(mon, x, y, width, label, value, pattern)
    local labelWidth = math.min(7, math.max(4, width - 8))
    local valueWidth = width - labelWidth - 1

    if valueWidth < 4 then
        draw.writeAt(mon, x, y, ("%s %s"):format(label, fmt(value, pattern)), colors.white, colors.black, width)
        return
    end

    draw.writeAt(mon, x, y, label, colors.lightGray, colors.black, labelWidth)
    draw.writeAt(mon, x + labelWidth + 1, y, cell(value, pattern, valueWidth), colors.white, colors.black, valueWidth)
end

local function drawValuePairRow(mon, x, y, width, left, right)
    local gap = 2
    local leftWidth = math.floor((width - gap) / 2)
    local rightWidth = width - leftWidth - gap

    if leftWidth < 12 or rightWidth < 12 then
        drawValueRow(mon, x, y, width, left.label, left.value, left.pattern)
        return
    end

    drawValueRow(mon, x, y, leftWidth, left.label, left.value, left.pattern)
    drawValueRow(mon, x + leftWidth + gap, y, rightWidth, right.label, right.value, right.pattern)
end

local function scaledOffset(value, limit, radius)
    local scaled = clamp(value / limit, -1.0, 1.0)
    return math.floor(scaled * radius + (scaled >= 0 and 0.5 or -0.5))
end

local function drawAttitudeColumn(mon, x, y, width, height, telemetry)
    if height < 1 then
        return
    end

    local state = expectTable(telemetry.state, "telemetry.state")
    local body = expectTable(state.body, "telemetry.state.body")
    local pose = expectTable(body.pose, "telemetry.state.body.pose")
    local rates = expectTable(body.rates, "telemetry.state.body.rates")
    local rows = {
        {
            { label = "ROLL", value = deg(pose.roll), pattern = "%+.1f" },
            { label = "RRATE", value = deg(rates.roll), pattern = "%+.1f" },
        },
        {
            { label = "PITCH", value = deg(pose.pitch), pattern = "%+.1f" },
            { label = "PRATE", value = deg(rates.pitch), pattern = "%+.1f" },
        },
        {
            { label = "HEAD", value = deg(pose.heading), pattern = "%+.1f" },
            { label = "YRATE", value = deg(rates.yaw), pattern = "%+.1f" },
        },
    }

    draw.writeAt(mon, x, y, "CURRENT", colors.lightGray, colors.black, width)

    for index, row in ipairs(rows) do
        local rowY = y + index

        if rowY >= y + height then
            return
        end

        drawValuePairRow(mon, x, rowY, width, row[1], row[2])
    end
end

local function drawPidOutputColumn(mon, x, y, width, height, rows, options)
    if height < 1 then
        return
    end

    options = options or {}

    if options.header then
        draw.writeAt(mon, x, y, "OUTPUT", colors.lightGray, colors.black, width)
        y = y + 1
        height = height - 1
    end

    for index, row in ipairs(rows) do
        local rowY = y + index - 1

        if rowY >= y + height then
            return
        end

        if options.labels == false then
            drawOutputBar(mon, x, rowY, width, row.value, row.limit)
        else
            drawOutput(mon, x, rowY, width, row.label, row.value, row.limit)
        end
    end
end

local function drawAttitudePid(mon, x, y, width, limitY, telemetry)
    if y > limitY then
        return y
    end

    section(mon, y, "attitude pid", colors.black, colors.lightBlue)
    y = y + 1

    local target = expectTable(telemetry.target, "telemetry.target")
    local current = expectTable(telemetry.current, "telemetry.current")
    local err = expectTable(telemetry.error, "telemetry.error")
    local output = expectTable(telemetry.output, "telemetry.output")
    local commands = expectTable(output.commands, "telemetry.output.commands")
    local pidData = expectTable(telemetry.pid, "telemetry.pid")
    local attitudePid = expectTable(pidData.attitude, "telemetry.pid.attitude")
    local targetAttitude = expectTable(target.attitude, "telemetry.target.attitude")
    local currentAttitude = expectTable(current.attitude, "telemetry.current.attitude")
    local errorAttitude = expectTable(err.attitude, "telemetry.error.attitude")
    local targetRoll = expectTable(targetAttitude.roll, "telemetry.target.attitude.roll")
    local targetPitch = expectTable(targetAttitude.pitch, "telemetry.target.attitude.pitch")
    local targetYaw = expectTable(targetAttitude.yaw, "telemetry.target.attitude.yaw")
    local currentRoll = expectTable(currentAttitude.roll, "telemetry.current.attitude.roll")
    local currentPitch = expectTable(currentAttitude.pitch, "telemetry.current.attitude.pitch")
    local currentYaw = expectTable(currentAttitude.yaw, "telemetry.current.attitude.yaw")
    local errorRoll = expectTable(errorAttitude.roll, "telemetry.error.attitude.roll")
    local errorPitch = expectTable(errorAttitude.pitch, "telemetry.error.attitude.pitch")
    local errorYaw = expectTable(errorAttitude.yaw, "telemetry.error.attitude.yaw")
    local outputRows = {
        { label = "COL", value = commands.collective, limit = 10.0 },
        { label = "ROL", value = commands.roll, limit = 8.0 },
        { label = "PIT", value = commands.pitch, limit = 12.0 },
        { label = "YAW", value = commands.yaw, limit = 8.0 },
    }
    local gap = PANEL_GAP
    local outputWidth = math.min(OUTPUT_COLUMN_WIDTH, math.max(MIN_OUTPUT_COLUMN_WIDTH, math.floor(width * 0.24)))
    local stateWidth = width - outputWidth - gap
    local topHeight = math.min(5, limitY - y + 1)

    drawAttitudeColumn(mon, x, y, stateWidth, topHeight, telemetry)
    drawPidOutputColumn(mon, x + stateWidth + gap, y, outputWidth, topHeight, outputRows, {
        header = true,
        labels = true,
    })

    y = y + topHeight

    if y > limitY then
        return y
    end

    drawControllerHeader(mon, x, y, width)

    if y + 1 <= limitY then drawControllerRow(mon, x, y + 1, width, "ROL", targetRoll.angle, currentRoll.angle, errorRoll.angle, true, attitudePid.roll.angle, true) end
    if y + 2 <= limitY then drawControllerRow(mon, x, y + 2, width, "RRAT", targetRoll.rate, currentRoll.rate, errorRoll.rate, true, attitudePid.roll.rate, false) end
    if y + 3 <= limitY then drawControllerRow(mon, x, y + 3, width, "PIT", targetPitch.angle, currentPitch.angle, errorPitch.angle, true, attitudePid.pitch.angle, true) end
    if y + 4 <= limitY then drawControllerRow(mon, x, y + 4, width, "PRAT", targetPitch.rate, currentPitch.rate, errorPitch.rate, true, attitudePid.pitch.rate, false) end
    if y + 5 <= limitY then drawControllerRow(mon, x, y + 5, width, "YAW", targetYaw.angle, currentYaw.angle, errorYaw.angle, true, attitudePid.yaw.angle, true) end
    if y + 6 <= limitY then drawControllerRow(mon, x, y + 6, width, "YRAT", targetYaw.rate, currentYaw.rate, errorYaw.rate, true, attitudePid.yaw.rate, false) end

    return y + math.min(7, limitY - y + 1)
end

local function drawPositionMap(mon, x, y, width, height, target, current)
    if height < 1 then
        return
    end

    if height > 2 and height % 2 == 0 then
        height = height - 1
    end

    local centerX = x + math.floor(width / 2)
    local centerY = y + math.floor(height / 2)
    local markX = centerX + scaledOffset(current.x - target.x, 10.0, math.floor((width - 3) / 2))
    local markY = centerY - scaledOffset(current.z - target.z, 10.0, math.floor((height - 1) / 2))

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

local function drawPositionHold(mon, x, y, width, limitY, telemetry)
    if y > limitY then
        return y
    end

    local positionHold = expectTable(telemetry.positionHold, "telemetry.positionHold")
    local worldPosition = expectTable(positionHold.worldPosition, "telemetry.positionHold.worldPosition")
    local worldVelocity = expectTable(positionHold.worldVelocity, "telemetry.positionHold.worldVelocity")
    local output = expectTable(positionHold.output, "telemetry.positionHold.output")
    local target = expectTable(worldPosition.target, "telemetry.positionHold.worldPosition.target")
    local currentPosition = expectTable(worldPosition.current, "telemetry.positionHold.worldPosition.current")
    local targetWorldVelocity = expectTable(worldVelocity.target, "telemetry.positionHold.worldVelocity.target")
    local currentWorldVelocity = expectTable(worldVelocity.current, "telemetry.positionHold.worldVelocity.current")
    local err = expectTable(worldPosition.error, "telemetry.positionHold.worldPosition.error")
    local pidData = expectTable(telemetry.pid, "telemetry.pid")
    local positionPid = expectTable(pidData.position, "telemetry.pid.position")
    local velocityPid = expectTable(pidData.velocity, "telemetry.pid.velocity")
    local positionXTerms = expectTable(positionPid.x, "telemetry.pid.position.x")
    local positionZTerms = expectTable(positionPid.z, "telemetry.pid.position.z")
    local velocityXTerms = expectTable(velocityPid.x, "telemetry.pid.velocity.x")
    local velocityZTerms = expectTable(velocityPid.z, "telemetry.pid.velocity.z")

    section(mon, y, "position hold", colors.black, colors.pink)
    y = y + 1

    local rows = {
        {
            label = "XPOS",
            target = 0.0,
            current = -err.x,
            err = err.x,
            terms = positionXTerms,
            angularTerms = false,
        },
        {
            label = "ZPOS",
            target = 0.0,
            current = -err.z,
            err = err.z,
            terms = positionZTerms,
            angularTerms = false,
        },
        {
            label = "XVEL",
            target = targetWorldVelocity.x,
            current = currentWorldVelocity.x,
            err = targetWorldVelocity.x - currentWorldVelocity.x,
            terms = velocityXTerms,
            angularTerms = true,
        },
        {
            label = "ZVEL",
            target = targetWorldVelocity.z,
            current = currentWorldVelocity.z,
            err = targetWorldVelocity.z - currentWorldVelocity.z,
            terms = velocityZTerms,
            angularTerms = true,
        },
    }
    local outputRows = {
        { label = "VX", value = targetWorldVelocity.x, limit = 20.0 },
        { label = "VZ", value = targetWorldVelocity.z, limit = 20.0 },
        { label = "ROL", value = deg(output.attitude.roll or 0.0), limit = 30.0 },
        { label = "PIT", value = deg(output.attitude.pitch or 0.0), limit = 30.0 },
    }
    local bodyHeight = math.min(5, limitY - y + 1)
    local layout = positionHoldLayout(x, width)

    drawPositionColumn(mon, layout.stateX, y, layout.stateWidth, bodyHeight, target, currentPosition)
    drawControllerHeader(mon, layout.pidX, y, layout.pidWidth)

    for index, row in ipairs(rows) do
        local rowY = y + index

        if rowY >= y + bodyHeight then
            break
        end

        drawControllerRow(
            mon,
            layout.pidX,
            rowY,
            layout.pidWidth,
            row.label,
            row.target,
            row.current,
            row.err,
            false,
            row.terms,
            row.angularTerms
        )
    end

    drawPidOutputColumn(mon, layout.outputX, y + 1, layout.outputWidth, bodyHeight - 1, outputRows, {
        labels = false,
    })

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
    local output = expectTable(telemetry.output, "telemetry.output")
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
        y = drawAttitudePid(mon, 2, y, w - 2, h - 2, telemetry)
    end

    if y <= h - 2 then
        y = y + 1
        y = drawPositionHold(mon, 2, y, w - 2, h - 2, telemetry)
    end

    if y <= h - 2 then
        y = y + 1
        y = drawRotorOutputs(mon, 2, y, w - 2, h - 2, output)
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
