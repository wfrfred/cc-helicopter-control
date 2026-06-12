local display_alloc = require("display_alloc")
local config = require("config")

local attitude_display = {}

local TEXT_SCALE = config.attitude.text_scale
local DRAW_DT = config.attitude.draw_dt

local PITCH_DEG_PER_ROW = config.attitude.pitch_deg_per_row
local CELL_ASPECT = config.attitude.cell_aspect

local ROLL_OFFSET_DEG = config.attitude.roll_offset_deg
local PITCH_OFFSET_DEG = config.attitude.pitch_offset_deg

local ROLL_LIMIT_DEG = config.attitude.roll_limit_deg
local PITCH_LIMIT_DEG = config.attitude.pitch_limit_deg

local CENTER_MARKER = config.attitude.center_marker

local function clamp(value, lo, hi)
    value = tonumber(value) or 0
    if value < lo then return lo end
    if value > hi then return hi end
    return value
end

local function deg(value)
    if value == nil then return 0 end
    return math.deg(value)
end

local function colorHex(color)
    if colors.toBlit then
        return colors.toBlit(color)
    end

    local index = 0
    while color > 1 do
        color = color / 2
        index = index + 1
    end

    return ("0123456789abcdef"):sub(index + 1, index + 1)
end

local function writeAt(mon, x, y, text, fg, bg)
    local w, h = mon.getSize()

    if y < 1 or y > h then return end

    text = tostring(text or "")

    if x < 1 then
        text = text:sub(2 - x)
        x = 1
    end

    if x > w or #text == 0 then return end

    local count = math.min(#text, w - x + 1)
    if count <= 0 then return end

    if mon.setTextColor then mon.setTextColor(fg or colors.white) end
    if mon.setBackgroundColor then mon.setBackgroundColor(bg or colors.black) end

    mon.setCursorPos(x, y)
    mon.write(text:sub(1, count))
end

local function blitRow(mon, x, y, text, fg, bg)
    local w, h = mon.getSize()

    if y < 1 or y > h then return end

    local textStr = table.concat(text)
    local fgStr = table.concat(fg)
    local bgStr = table.concat(bg)

    if x < 1 then
        local cut = 2 - x
        textStr = textStr:sub(cut)
        fgStr = fgStr:sub(cut)
        bgStr = bgStr:sub(cut)
        x = 1
    end

    if x > w or #textStr == 0 then return end

    local count = math.min(#textStr, w - x + 1)
    if count <= 0 then return end

    textStr = textStr:sub(1, count)
    fgStr = fgStr:sub(1, count)
    bgStr = bgStr:sub(1, count)

    mon.setCursorPos(x, y)

    if mon.blit then
        mon.blit(textStr, fgStr, bgStr)
    else
        mon.write(textStr)
    end
end

local function makeRow(width)
    local text, fg, bg = {}, {}, {}
    local white = colorHex(colors.white)
    local black = colorHex(colors.black)

    for i = 1, width do
        text[i] = " "
        fg[i] = white
        bg[i] = black
    end

    return text, fg, bg
end

local function setCell(text, fg, bg, index, ch, fgColor, bgColor)
    if index < 1 or index > #text then return end

    text[index] = ch or " "

    if fgColor then
        fg[index] = colorHex(fgColor)
    end

    if bgColor then
        bg[index] = colorHex(bgColor)
    end
end

local function centerOf(width, height)
    return math.floor((width + 1) / 2), math.floor((height + 1) / 2)
end

local function getAttitude(current)
    local roll = deg(current.roll) + ROLL_OFFSET_DEG
    local pitch = -deg(current.pitch) + PITCH_OFFSET_DEG

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
        local text, fg, bg = makeRow(width)

        for col = 1, width do
            local horizonY = centerY + pitchOffset + (col - centerX) * slope
            local dist = row - horizonY

            if math.abs(dist) <= 0.35 then
                setCell(text, fg, bg, col, "-", colors.white, colors.gray)
            elseif row < horizonY then
                local sky = math.abs(dist) < 1.2 and colors.cyan or colors.lightBlue
                setCell(text, fg, bg, col, " ", colors.white, sky)
            else
                local ground = math.abs(dist) < 1.2 and colors.orange or colors.brown
                setCell(text, fg, bg, col, " ", colors.white, ground)
            end
        end

        blitRow(mon, 1, row, text, fg, bg)
    end

    local markerX = centerX - math.floor(#CENTER_MARKER / 2)
    writeAt(mon, markerX, centerY, CENTER_MARKER, colors.black, colors.yellow)
end

local function draw(mon, shared)
    local telemetry = shared.telemetry or {}
    local current = telemetry.current or {}

    drawHorizon(mon, current)
end

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
