local draw = require("draw")

local monitor_view = {}

local STALE_TELEMETRY_DT = 0.5
local TAB_ROW = 3
local CONTENT_TOP = 4
local GAP = 2
local BG = colors.black
local SURFACE = colors.gray
local MUTED = colors.lightGray
local TEXT = colors.white
local CURRENT = colors.lightBlue
local TARGET = colors.cyan
local ACTIVE = colors.green
local SELECTED = colors.gray
local PENDING = colors.orange
local DANGER = colors.red
local HEADER = colors.lightGray

local PAGES = {
    { id = "overview", label = "OVERVIEW" },
    { id = "attitude", label = "ATT PID" },
    { id = "position", label = "POS PID" },
    { id = "nav", label = "NAV" },
}

local function clamp(x, lo, hi)
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

local function deg(x)
    return math.deg(x or 0.0)
end

local function fmt(value, pattern)
    return (pattern or "%.1f"):format(value or 0.0)
end

local function clip(text, width)
    return draw.clip(tostring(text), width)
end

local function rightText(text, width)
    local out = clip(text, width)
    return string.rep(" ", math.max(0, width - #out)) .. out
end

local function statusColor(value)
    if value == true or value == "active" or value == "locked" or value == "running" then
        return ACTIVE
    end

    if value == "pending" or value == "transit" or value == "turn" or value == "climb" or value == "descend" or value == "final_approach" then
        return PENDING
    end

    if value == "error" or value == "stale" then
        return DANGER
    end

    return SELECTED
end

local function pageId(value)
    for _, page in ipairs(PAGES) do
        if value == page.id then
            return value
        end
    end

    return PAGES[1].id
end

local function pageAt(mon, x, y)
    if y ~= TAB_ROW then
        return nil
    end

    local w = mon.getSize()
    local tabWidth = math.max(1, math.floor(w / #PAGES))
    local index = math.floor((x - 1) / tabWidth) + 1

    if index < 1 then
        index = 1
    elseif index > #PAGES then
        index = #PAGES
    end

    return PAGES[index].id
end

local function section(mon, y, title, fg, bg)
    local w = mon.getSize()

    if y < 1 then
        return
    end

    draw.writeAt(mon, 1, y, string.upper(title), fg or colors.black, bg or HEADER, w)
end

local function drawFooter(mon, shared, telemetry, staleTelemetry)
    local w, h = mon.getSize()

    if h < 3 then
        return
    end

    local footer = ("seq %d  telemetry %s"):format(
        shared.inputSeq,
        tostring(shared.telemetrySender)
    )

    if staleTelemetry then
        footer = footer .. " STALE"
    end

    draw.writeAt(mon, 1, h - 1, footer, MUTED, BG, w)
    draw.writeAt(mon, 1, h, "touch tabs to switch pages", TEXT, SURFACE, w)
end

local function drawValue(mon, x, y, width, label, value, pattern)
    local labelWidth = math.min(7, math.max(4, width - 8))
    local valueWidth = width - labelWidth - 1

    if valueWidth < 4 then
        draw.writeAt(mon, x, y, ("%s %s"):format(label, fmt(value, pattern)), TEXT, BG, width)
        return
    end

    draw.writeAt(mon, x, y, label, MUTED, BG, labelWidth)
    draw.writeAt(mon, x + labelWidth + 1, y, rightText(fmt(value, pattern), valueWidth), TEXT, BG, valueWidth)
end

local function drawValueGrid(mon, x, y, width, rows)
    local colWidth = math.floor((width - GAP) / 2)

    for index, row in ipairs(rows) do
        local col = (index - 1) % 2
        local rowY = y + math.floor((index - 1) / 2)
        local rowX = x + col * (colWidth + GAP)
        local rowWidth = col == 0 and colWidth or (width - colWidth - GAP)

        drawValue(mon, rowX, rowY, rowWidth, row.label, row.value, row.pattern)
    end
end

local function waypointLabel(waypoint, width)
    local position = waypoint.position
    local name = waypoint.name or waypoint.id
    local text = ("%s  X%.0f Y%.0f Z%.0f"):format(name, position.x, position.y, position.z)

    return clip(text, width)
end

local function drawWaypointRows(mon, shared, x, y, width, limitY, navigation)
    local waypoints = navigation.waypoints
    local selected = navigation.selected
    local active = navigation.active == true and navigation.waypoint or nil
    local touch = {}

    if #waypoints == 0 then
        draw.writeAt(mon, x, y, "no waypoints", colors.lightGray, colors.black, width)
        shared.monitorTouch.navRows = touch
        return y + 1
    end

    for index, waypoint in ipairs(waypoints) do
        local rowY = y + index - 1

        if rowY > limitY then
            break
        end

        local isActive = active ~= nil and active.id == waypoint.id
        local isSelected = selected ~= nil and selected.id == waypoint.id
        local fg = TEXT
        local bg = BG
        local prefix = "  "

        if isActive then
            fg = colors.black
            bg = ACTIVE
            prefix = "> "
        elseif isSelected then
            fg = TEXT
            bg = SELECTED
            prefix = "* "
        end

        draw.writeAt(mon, x, rowY, prefix .. waypointLabel(waypoint, math.max(0, width - 2)), fg, bg, width)
        touch[#touch + 1] = {
            y = rowY,
            x1 = x,
            x2 = x + width - 1,
            waypoint = waypoint.id,
        }
    end

    shared.monitorTouch.navRows = touch

    return y + #touch
end

local function horizontalDistance(a, b)
    local dx = (b.x or 0.0) - (a.x or 0.0)
    local dz = (b.z or 0.0) - (a.z or 0.0)

    return math.sqrt(dx * dx + dz * dz)
end

local function verticalError(current, target)
    return (target.height or target.y or 0.0) - (current.y or 0.0)
end

local function drawStatusLine(mon, x, y, width, label, value, color)
    local labelWidth = math.min(10, math.max(4, width - 10))
    local valueWidth = width - labelWidth - 1

    if valueWidth < 4 then
        draw.writeAt(mon, x, y, ("%s %s"):format(label, tostring(value)), TEXT, BG, width)
        return
    end

    draw.writeAt(mon, x, y, label, MUTED, BG, labelWidth)
    draw.writeAt(mon, x + labelWidth + 1, y, rightText(tostring(value), valueWidth), colors.black, color, valueWidth)
end

local function navPhaseLabel(navigation)
    if navigation.active == true then
        return tostring(navigation.phase or "active")
    end

    if navigation.selected ~= nil then
        return "selected"
    end

    return tostring(navigation.reason or "idle")
end

local function drawOutput(mon, x, y, width, label, value, limit)
    local labelWidth = width >= 24 and 5 or 4
    local valueWidth = 7
    local barWidth = width - labelWidth - valueWidth - 2

    if barWidth < 6 then
        draw.writeAt(mon, x, y, ("%s %s"):format(label, fmt(value, "%+.1f")), colors.white, colors.black, width)
        return
    end

    local pct = math.abs(clamp((value or 0.0) / limit, -1.0, 1.0))
    local len = math.floor(barWidth * pct + 0.5)
    local bg = (value or 0.0) >= 0 and colors.blue or colors.purple
    local barX = x + labelWidth + 1

    draw.writeAt(mon, x, y, label, colors.lightGray, colors.black, labelWidth)
    draw.fill(mon, barX, y, barWidth, colors.gray)
    draw.fill(mon, barX, y, len, bg)
    draw.writeAt(mon, barX + barWidth + 1, y, fmt(value, "%+.1f"), colors.white, colors.black, valueWidth)
end

local function drawOutputGrid(mon, x, y, width, limitY, rows)
    local columns = width >= 64 and 2 or 1
    local colWidth = columns == 2 and math.floor((width - GAP) / 2) or width
    local rowY = y

    for index, row in ipairs(rows) do
        local col = (index - 1) % columns

        if col == 0 and index > 1 then
            rowY = rowY + 1
        end

        if rowY > limitY then
            return rowY
        end

        local rowX = x + col * (colWidth + GAP)
        local rowWidth = col == 0 and colWidth or (width - colWidth - GAP)
        drawOutput(mon, rowX, rowY, rowWidth, row.label, row.value, row.limit)
    end

    return rowY + 1
end

local function drawRotorOutputs(mon, x, y, width, limitY, telemetry)
    local rotor = telemetry.rotor
    local upper = rotor.upper
    local lower = rotor.lower
    local rows = {
        { label = "UF", value = upper[1], limit = 15.0 },
        { label = "UR", value = upper[2], limit = 15.0 },
        { label = "UB", value = upper[3], limit = 15.0 },
        { label = "UL", value = upper[4], limit = 15.0 },
        { label = "LF", value = lower[1], limit = 15.0 },
        { label = "LR", value = lower[2], limit = 15.0 },
        { label = "LB", value = lower[3], limit = 15.0 },
        { label = "LL", value = lower[4], limit = 15.0 },
    }

    if y > limitY then
        return y
    end

    section(mon, y, "blade outputs", colors.black, HEADER)
    return drawOutputGrid(mon, x, y + 1, width, limitY, rows)
end

local function pidTableSpec(width)
    if width >= 62 then
        return {
            labels = { "AXIS", "TARGET", "CURRENT", "ERROR", "P", "I", "D", "OUT" },
            widths = { 5, 7, 7, 7, 7, 7, 7, 7 },
            valuePattern = "%.1f",
            termPattern = "%+.1f",
        }
    end

    if width >= 48 then
        return {
            labels = { "AXIS", "TGT", "CUR", "ERR", "P", "I", "D", "OUT" },
            widths = { 5, 6, 6, 6, 5, 5, 5, 6 },
            valuePattern = "%.1f",
            termPattern = "%+.1f",
        }
    end

    return {
        labels = { "AX", "TGT", "CUR", "ERR", "P", "I", "D", "OUT" },
        widths = { 4, 5, 5, 5, 4, 4, 4, 5 },
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
        local text = index == 1 and clip(value, column.width) or rightText(value, column.width)

        draw.writeAt(mon, x + column.offset, y, text, fg, colors.black, column.width)
    end
end

local function drawPidHeader(mon, x, y, width)
    drawPidTableCells(mon, x, y, width, pidTableSpec(width).labels, colors.lightGray)
end

local function drawPidRow(mon, x, y, width, label, target, current, err, angle, terms, angularTerms)
    if angle then
        target = deg(target)
        current = deg(current)
        err = deg(err)
    end

    local p = terms.p
    local i = terms.i
    local d = terms.d
    local output = terms.output

    if angularTerms then
        p = deg(p)
        i = deg(i)
        d = deg(d)
        output = deg(output)
    end

    local spec = pidTableSpec(width)

    drawPidTableCells(mon, x, y, width, {
        label,
        fmt(target, spec.valuePattern),
        fmt(current, spec.valuePattern),
        fmt(err, spec.valuePattern),
        fmt(p, spec.termPattern),
        fmt(i, spec.termPattern),
        fmt(d, spec.termPattern),
        fmt(output, spec.termPattern),
    }, colors.white)
end

local function scaledOffset(value, limit, radius)
    local scaled = clamp(value / limit, -1.0, 1.0)
    return math.floor(scaled * radius + (scaled >= 0 and 0.5 or -0.5))
end

local function drawPositionMap(mon, x, y, width, height, target, current)
    if height < 1 or width < 5 then
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

local function attitudeOutputRows(telemetry)
    local commands = telemetry.command

    return {
        { label = "COL", value = commands.collective, limit = 10.0 },
        { label = "ROL", value = commands.roll, limit = 8.0 },
        { label = "PIT", value = commands.pitch, limit = 12.0 },
        { label = "YAW", value = commands.yaw, limit = 8.0 },
    }
end

local function positionOutputRows(telemetry)
    local control = telemetry.control
    local horizontal = control.horizontal
    local velocity = horizontal.velocity or {
        target = {},
    }
    local output = horizontal.output or {
        angle = {},
    }
    local targetVelocity = velocity.target

    return {
        { label = "VF", value = targetVelocity.forward, limit = 20.0 },
        { label = "VR", value = targetVelocity.right, limit = 20.0 },
        { label = "ROL", value = deg(output.angle.roll or 0.0), limit = 30.0 },
        { label = "PIT", value = deg(output.angle.pitch or 0.0), limit = 30.0 },
    }
end

local function mapPointFromHorizontal(position)
    return {
        x = position.right or 0.0,
        z = -(position.forward or 0.0),
    }
end

local function drawCurrentAttitude(mon, x, y, width, telemetry)
    local state = telemetry.state
    local body = state.body
    local pose = body.pose
    local angular = body.angular
    local rates = angular.velocity

    drawValueGrid(mon, x, y, width, {
        { label = "ROLL", value = deg(pose.roll), pattern = "%+.1f" },
        { label = "RRATE", value = deg(rates.roll), pattern = "%+.1f" },
        { label = "PITCH", value = deg(pose.pitch), pattern = "%+.1f" },
        { label = "PRATE", value = deg(rates.pitch), pattern = "%+.1f" },
        { label = "HEAD", value = deg(pose.heading), pattern = "%+.1f" },
        { label = "YRATE", value = deg(rates.yaw), pattern = "%+.1f" },
    })
end

local function drawTargetAttitude(mon, x, y, width, telemetry)
    local control = telemetry.control
    local attitude = control.attitude
    local target = attitude.target
    local roll = target.roll
    local pitch = target.pitch
    local yaw = target.yaw

    drawValueGrid(mon, x, y, width, {
        { label = "ROLL", value = deg(roll.angle), pattern = "%+.1f" },
        { label = "RRATE", value = deg(roll.rate), pattern = "%+.1f" },
        { label = "PITCH", value = deg(pitch.angle), pattern = "%+.1f" },
        { label = "PRATE", value = deg(pitch.rate), pattern = "%+.1f" },
        { label = "YAW", value = deg(yaw.angle), pattern = "%+.1f" },
        { label = "YRATE", value = deg(yaw.rate), pattern = "%+.1f" },
    })
end

local function drawOverview(mon, x, y, width, limitY, telemetry)
    local control = telemetry.control
    local horizontal = control.horizontal

    section(mon, y, "flight state", colors.black, CURRENT)
    y = y + 1
    drawCurrentAttitude(mon, x, y, width, telemetry)
    y = y + 3

    if y <= limitY then
        section(mon, y, "attitude output", colors.black, TARGET)
        y = drawOutputGrid(mon, x, y + 1, width, limitY, attitudeOutputRows(telemetry))
    end

    if y <= limitY then
        y = y + 1
        section(mon, y, "position hold", colors.black, TARGET)
        y = y + 1

        if horizontal.kind == "position" then
            local position = horizontal.position
            local targetPosition = mapPointFromHorizontal(position.target)
            local currentPosition = mapPointFromHorizontal(position.current)
            local mapWidth = math.min(24, math.max(12, math.floor(width * 0.25)))
            local outputX = x + mapWidth + GAP
            local outputWidth = width - mapWidth - GAP
            local mapHeight = math.min(5, math.max(3, limitY - y + 1))

            drawPositionMap(mon, x, y, mapWidth, mapHeight, targetPosition, currentPosition)
            drawOutputGrid(mon, outputX, y, outputWidth, limitY, positionOutputRows(telemetry))
            y = y + mapHeight
        else
            y = drawOutputGrid(mon, x, y, width, limitY, positionOutputRows(telemetry))
        end
    end

    if y <= limitY then
        y = y + 1
        y = drawRotorOutputs(mon, x, y, width, limitY, telemetry)
    end

    return y
end

local function drawAttitudePid(mon, x, y, width, limitY, telemetry)
    local control = telemetry.control
    local attitude = control.attitude
    local targetAttitude = attitude.target
    local currentAttitude = attitude.current
    local errorAttitude = attitude.error
    local attitudePid = attitude.pid
    local targetRoll = targetAttitude.roll
    local targetPitch = targetAttitude.pitch
    local targetYaw = targetAttitude.yaw
    local currentRoll = currentAttitude.roll
    local currentPitch = currentAttitude.pitch
    local currentYaw = currentAttitude.yaw
    local errorRoll = errorAttitude.roll
    local errorPitch = errorAttitude.pitch
    local errorYaw = errorAttitude.yaw

    section(mon, y, "current attitude", colors.black, CURRENT)
    y = y + 1
    drawCurrentAttitude(mon, x, y, width, telemetry)
    y = y + 3

    if y <= limitY then
        section(mon, y, "target attitude", colors.black, TARGET)
        y = y + 1
        drawTargetAttitude(mon, x, y, width, telemetry)
        y = y + 3
    end

    if y <= limitY then
        section(mon, y, "attitude output", colors.black, TARGET)
        y = drawOutputGrid(mon, x, y + 1, width, limitY, attitudeOutputRows(telemetry))
    end

    if y <= limitY then
        y = y + 1
        section(mon, y, "attitude angle pid", colors.black, HEADER)
        y = y + 1
        drawPidHeader(mon, x, y, width)
        if y + 1 <= limitY then drawPidRow(mon, x, y + 1, width, "RANG", targetRoll.angle, currentRoll.angle, errorRoll.angle, true, attitudePid.roll.angle, false) end
        if y + 2 <= limitY then drawPidRow(mon, x, y + 2, width, "PANG", targetPitch.angle, currentPitch.angle, errorPitch.angle, true, attitudePid.pitch.angle, false) end
        if y + 3 <= limitY then drawPidRow(mon, x, y + 3, width, "YANG", targetYaw.angle, currentYaw.angle, errorYaw.angle, true, attitudePid.yaw.angle, false) end
        y = y + 4
    end

    if y <= limitY then
        y = y + 1
        section(mon, y, "attitude rate pid", colors.black, HEADER)
        y = y + 1
        drawPidHeader(mon, x, y, width)
        if y + 1 <= limitY then drawPidRow(mon, x, y + 1, width, "RRAT", targetRoll.rate, currentRoll.rate, errorRoll.rate, true, attitudePid.roll.rate, false) end
        if y + 2 <= limitY then drawPidRow(mon, x, y + 2, width, "PRAT", targetPitch.rate, currentPitch.rate, errorPitch.rate, true, attitudePid.pitch.rate, false) end
        if y + 3 <= limitY then drawPidRow(mon, x, y + 3, width, "YRAT", targetYaw.rate, currentYaw.rate, errorYaw.rate, true, attitudePid.yaw.rate, false) end
        y = y + 4
    end

    return y
end

local function drawPositionPid(mon, x, y, width, limitY, telemetry)
    local control = telemetry.control
    local horizontal = control.horizontal

    if horizontal.kind ~= "position" then
        section(mon, y, "horizontal attitude", colors.black, TARGET)
        return drawOutputGrid(mon, x, y + 1, width, limitY, positionOutputRows(telemetry))
    end

    local position = horizontal.position
    local velocity = horizontal.velocity
    local mapTargetPosition = mapPointFromHorizontal(position.target)
    local mapCurrentPosition = mapPointFromHorizontal(position.current)
    local targetPosition = position.target
    local currentPosition = position.current
    local errorPosition = position.error
    local targetVelocity = velocity.target
    local currentVelocity = velocity.current
    local errorVelocity = velocity.error
    local positionPid = horizontal.pid.position
    local velocityPid = horizontal.pid.velocity

    section(mon, y, "position output", colors.black, TARGET)
    y = y + 1

    local mapWidth = math.min(24, math.max(12, math.floor(width * 0.25)))
    local outputX = x + mapWidth + GAP
    local outputWidth = width - mapWidth - GAP
    local mapHeight = math.min(5, math.max(1, limitY - y + 1))

    drawPositionMap(mon, x, y, mapWidth, mapHeight, mapTargetPosition, mapCurrentPosition)
    drawOutputGrid(mon, outputX, y, outputWidth, limitY, positionOutputRows(telemetry))
    y = y + mapHeight

    if y <= limitY then
        y = y + 1
        section(mon, y, "position hold pid", colors.black, HEADER)
        y = y + 1
        drawPidHeader(mon, x, y, width)

        if y + 1 <= limitY then
            drawPidRow(
                mon,
                x,
                y + 1,
                width,
                "FPOS",
                targetPosition.forward,
                currentPosition.forward,
                errorPosition.forward,
                false,
                positionPid.forward,
                false
            )
        end
        if y + 2 <= limitY then
            drawPidRow(
                mon,
                x,
                y + 2,
                width,
                "RPOS",
                targetPosition.right,
                currentPosition.right,
                errorPosition.right,
                false,
                positionPid.right,
                false
            )
        end
        if y + 3 <= limitY then
            drawPidRow(
                mon,
                x,
                y + 3,
                width,
                "FVEL",
                targetVelocity.forward,
                currentVelocity.forward,
                errorVelocity.forward,
                false,
                velocityPid.forward,
                true
            )
        end
        if y + 4 <= limitY then
            drawPidRow(
                mon,
                x,
                y + 4,
                width,
                "RVEL",
                targetVelocity.right,
                currentVelocity.right,
                errorVelocity.right,
                false,
                velocityPid.right,
                true
            )
        end
        y = y + 5
    end

    return y
end

local function drawNavigation(mon, x, y, width, limitY, telemetry, shared)
    local state = telemetry.state
    local raw = state.raw
    local position = raw.position
    local velocity = raw.velocity
    local heading = telemetry.heading
    local navigation = telemetry.navigation
    local navTarget = navigation.target
    local waypoint = navigation.waypoint or navigation.selected
    local approach = navigation.approach
    local hdist = navTarget and navTarget.position and horizontalDistance(position, navTarget.position) or 0.0
    local verr = navTarget and verticalError(position, navTarget) or 0.0
    local navColor = navigation.active and statusColor(navigation.phase) or statusColor(navPhaseLabel(navigation))

    section(mon, y, "navigation", colors.black, navColor)
    y = y + 1

    if y <= limitY then
        local waypointName = waypoint and (waypoint.name or waypoint.id) or "-"
        local approachName = approach and (approach.name or approach.id) or "-"

        drawStatusLine(mon, x, y, math.floor((width - GAP) / 2), "STATE", navPhaseLabel(navigation), navColor)
        drawStatusLine(mon, x + math.floor((width - GAP) / 2) + GAP, y, width - math.floor((width - GAP) / 2) - GAP, "TARGET", waypointName, navigation.active and ACTIVE or SELECTED)
        if y + 1 <= limitY then
            draw.writeAt(mon, x, y + 1, "approach " .. tostring(approachName), MUTED, BG, width)
        end
        y = y + 2
    end

    if y <= limitY then
        section(mon, y, "current", colors.black, CURRENT)
        y = y + 1
    end

    drawValueGrid(mon, x, y, width, {
        { label = "X", value = position.x, pattern = "%.1f" },
        { label = "Z", value = position.z, pattern = "%.1f" },
        { label = "ALT", value = position.y, pattern = "%.1f" },
        { label = "VY", value = velocity.y, pattern = "%+.1f" },
        { label = "HEAD", value = deg(heading.angle), pattern = "%+.1f" },
        { label = "HERR", value = deg(heading.error), pattern = "%+.1f" },
    })
    y = y + 3

    if y <= limitY then
        y = y + 1
        section(mon, y, "target", colors.black, TARGET)
        y = y + 1

        if navTarget ~= nil and navTarget.position ~= nil then
            drawValueGrid(mon, x, y, width, {
                { label = "TX", value = navTarget.position.x, pattern = "%.1f" },
                { label = "TZ", value = navTarget.position.z, pattern = "%.1f" },
                { label = "TALT", value = navTarget.height, pattern = "%.1f" },
                { label = "THEAD", value = deg(navTarget.heading), pattern = "%+.1f" },
                { label = "HDIST", value = hdist, pattern = "%.1f" },
                { label = "VERR", value = verr, pattern = "%+.1f" },
            })
            y = y + 3
        else
            draw.writeAt(mon, x, y, "no active target", MUTED, BG, width)
            y = y + 1
        end
    end

    if y <= limitY then
        y = y + 1
        section(mon, y, "waypoints", colors.black, HEADER)
        y = drawWaypointRows(mon, shared, x, y + 1, width, limitY, navigation)
    end

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

local pageDrawers = {
    overview = drawOverview,
    attitude = drawAttitudePid,
    position = drawPositionPid,
    nav = drawNavigation,
}

local function drawRunning(mon, shared, telemetry)
    local w, h = mon.getSize()
    local now = os.clock()
    local activePage = pageId(shared.monitorPage)
    local telemetryAge = now - shared.telemetryTime
    local inputTelemetry = telemetry.input
    local staleTelemetry = telemetryAge > STALE_TELEMETRY_DT or inputTelemetry.stale
    local stateColor = staleTelemetry and colors.orange or colors.green

    shared.monitorPage = activePage

    draw.clear(mon, colors.black)
    draw.writeAt(mon, 1, 1, " HELI INPUT / DISPLAY", colors.black, colors.lime, w)
    draw.writeAt(mon, 1, 2, " running", colors.black, stateColor, math.min(w, 18))
    draw.writeAt(
        mon,
        20,
        2,
        ("ctl %.2fs  in %.2fs"):format(telemetryAge, inputTelemetry.age or 0.0),
        colors.lightGray,
        colors.black,
        math.max(0, w - 19)
    )

    local tabWidth = math.max(1, math.floor(w / #PAGES))
    for index, page in ipairs(PAGES) do
        local x = (index - 1) * tabWidth + 1
        local width = index == #PAGES and (w - x + 1) or tabWidth
        local active = page.id == activePage

        draw.writeAt(
            mon,
            x,
            TAB_ROW,
            " " .. page.label,
            active and colors.black or colors.white,
            active and colors.lime or colors.gray,
            width
        )
    end

    shared.monitorTouch = {
        page = activePage,
        navRows = {},
    }

    local drawer = pageDrawers[activePage] or drawOverview
    drawer(mon, 2, CONTENT_TOP, w - 2, h - 2, telemetry, shared)
    drawFooter(mon, shared, telemetry, staleTelemetry)
end

function monitor_view.draw(mon, shared)
    local telemetry = shared.telemetry

    if telemetry == nil then
        drawWaiting(mon, shared)
        return
    end

    if telemetry.status ~= "running" then
        drawNonRunning(mon, shared, telemetry)
        return
    end

    drawRunning(mon, shared, telemetry)
end

function monitor_view.handleTouch(mon, shared, x, y)
    local page = pageAt(mon, x, y)

    if page then
        shared.monitorPage = page
        return true
    end

    local touch = shared.monitorTouch

    if touch ~= nil and touch.page == "nav" then
        for _, row in ipairs(touch.navRows) do
            if y == row.y and x >= row.x1 and x <= row.x2 then
                local telemetry = shared.telemetry
                local navigation = telemetry.navigation
                local activeWaypoint = navigation.active == true and navigation.waypoint or nil

                if activeWaypoint ~= nil and activeWaypoint.id == row.waypoint then
                    shared.pendingNavigationCommand = {
                        action = "cancel",
                    }
                    return true
                end

                shared.pendingNavigationCommand = {
                    action = "activate",
                    waypoint = row.waypoint,
                }
                return true
            end
        end
    end

    return false
end

return monitor_view
