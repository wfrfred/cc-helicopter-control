local draw = require("draw")
local config = require("config")

local attitude_view = {}

local PITCH_DEG_PER_ROW = config.attitude.pitch_deg_per_row
local CELL_ASPECT = config.attitude.cell_aspect

local ROLL_OFFSET_DEG = config.attitude.roll_offset_deg
local PITCH_OFFSET_DEG = config.attitude.pitch_offset_deg

local ROLL_LIMIT_DEG = config.attitude.roll_limit_deg
local PITCH_LIMIT_DEG = config.attitude.pitch_limit_deg

local CENTER_MARKER = config.attitude.center_marker

local function clamp(value, lo, hi)
    if value < lo then return lo end
    if value > hi then return hi end
    return value
end

local function expectTable(value, name)
    assert(type(value) == "table", name .. " must be table")
    return value
end

local function centerOf(width, height)
    return math.floor((width + 1) / 2), math.floor((height + 1) / 2)
end

local function getAttitude(current)
    local roll = math.deg(current.roll) + ROLL_OFFSET_DEG
    local pitch = -math.deg(current.pitch) + PITCH_OFFSET_DEG

    return clamp(roll, -ROLL_LIMIT_DEG, ROLL_LIMIT_DEG),
           clamp(pitch, -PITCH_LIMIT_DEG, PITCH_LIMIT_DEG)
end

local function drawHorizon(mon, current)
    local width, height = mon.getSize()
    if width <= 0 or height <= 0 then return end

    local centerX, centerY = centerOf(width, height)
    local roll, pitch = getAttitude(current)

    local slope = math.tan(math.rad(roll)) * CELL_ASPECT
    local pitchOffset = pitch / PITCH_DEG_PER_ROW

    for row = 1, height do
        local text, fg, bg = draw.makeRow(width, colors.white, colors.black)

        for col = 1, width do
            local horizonY = centerY + pitchOffset + (col - centerX) * slope
            local dist = row - horizonY

            if math.abs(dist) <= 0.35 then
                draw.setCell(text, fg, bg, col, "-", colors.white, colors.gray)
            elseif row < horizonY then
                local sky = math.abs(dist) < 1.2 and colors.cyan or colors.lightBlue
                draw.setCell(text, fg, bg, col, " ", colors.white, sky)
            else
                local ground = math.abs(dist) < 1.2 and colors.orange or colors.brown
                draw.setCell(text, fg, bg, col, " ", colors.white, ground)
            end
        end

        draw.blitRow(mon, 1, row, text, fg, bg)
    end

    local markerX = centerX - math.floor(#CENTER_MARKER / 2)
    draw.writeAt(mon, markerX, centerY, CENTER_MARKER, colors.black, colors.yellow, #CENTER_MARKER)
end

local function drawWaiting(mon, text)
    local w, h = mon.getSize()
    draw.clear(mon, colors.black)
    draw.writeAt(mon, 1, math.max(1, math.floor(h / 2)), text, colors.white, colors.black, w)
end

function attitude_view.draw(mon, shared)
    local telemetry = shared.telemetry

    if telemetry == nil then
        drawWaiting(mon, "waiting for telemetry")
        return
    end

    expectTable(telemetry, "shared.telemetry")

    if telemetry.status ~= "running" then
        drawWaiting(mon, "status " .. telemetry.status)
        return
    end

    local current = expectTable(telemetry.current, "telemetry.current")
    drawHorizon(mon, expectTable(current.attitude, "telemetry.current.attitude"))
end

return attitude_view
