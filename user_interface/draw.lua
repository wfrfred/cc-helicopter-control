local draw = {}

function draw.colorHex(color)
    return colors.toBlit(color)
end

function draw.clip(text, width)
    if #text > width then
        return text:sub(1, width)
    end

    return text
end

function draw.setFg(mon, color)
    mon.setTextColor(color)
end

function draw.setBg(mon, color)
    mon.setBackgroundColor(color)
end

function draw.clear(mon, bg)
    draw.setBg(mon, bg)
    mon.clear()
end

function draw.writeAt(mon, x, y, text, fg, bg, width)
    local w, h = mon.getSize()

    if y < 1 or y > h or x > w then
        return
    end

    width = width or (w - x + 1)
    width = math.min(width, w - x + 1)

    if width <= 0 then
        return
    end

    local out = draw.clip(text, width)

    draw.setFg(mon, fg)
    draw.setBg(mon, bg)
    mon.setCursorPos(x, y)
    mon.write(out .. string.rep(" ", width - #out))
end

function draw.fill(mon, x, y, width, bg)
    local w, h = mon.getSize()

    if y < 1 or y > h then
        return
    end

    if x < 1 then
        width = width + x - 1
        x = 1
    end

    if x > w then
        return
    end

    width = math.min(width, w - x + 1)
    if width <= 0 then
        return
    end

    draw.setBg(mon, bg)
    mon.setCursorPos(x, y)
    mon.write(string.rep(" ", width))
end

function draw.makeRow(width, fgColor, bgColor)
    local text, fg, bg = {}, {}, {}
    local fgHex = draw.colorHex(fgColor)
    local bgHex = draw.colorHex(bgColor)

    for i = 1, width do
        text[i] = " "
        fg[i] = fgHex
        bg[i] = bgHex
    end

    return text, fg, bg
end

function draw.setCell(text, fg, bg, index, ch, fgColor, bgColor)
    if index < 1 or index > #text then
        return
    end

    text[index] = ch

    if fgColor then
        fg[index] = draw.colorHex(fgColor)
    end

    if bgColor then
        bg[index] = draw.colorHex(bgColor)
    end
end

function draw.blitRow(mon, x, y, text, fg, bg)
    local w, h = mon.getSize()

    if y < 1 or y > h then
        return
    end

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

    if x > w or #textStr == 0 then
        return
    end

    local count = math.min(#textStr, w - x + 1)
    if count <= 0 then
        return
    end

    mon.setCursorPos(x, y)
    mon.blit(textStr:sub(1, count), fgStr:sub(1, count), bgStr:sub(1, count))
end

return draw
