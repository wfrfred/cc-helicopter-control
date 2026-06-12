local display_alloc = {}

local function normalizeName(name)
    if name == nil or name == "" or name == "-" then
        return nil
    end

    return name
end

local function usableMonitor(name)
    local mon = peripheral.wrap(name)

    if mon and mon.getSize and mon.setCursorPos and mon.write then
        local w, h = mon.getSize()

        return {
            name = name,
            mon = mon,
            w = w,
            h = h,
            area = w * h,
        }
    end
end

local function configured(shared, role)
    local displays = shared.displays or {}
    local name = normalizeName(displays[role])

    if not name then
        return nil
    end

    local info = usableMonitor(name)

    if info then
        return info.mon, info.name
    end
end

local function listMonitors()
    local out = {}

    if not peripheral.getNames then
        local mon = peripheral.find("monitor")
        if mon then
            local w, h = mon.getSize()

            out[1] = {
                name = "monitor",
                mon = mon,
                w = w,
                h = h,
                area = w * h,
            }
        end
        return out
    end

    for _, name in ipairs(peripheral.getNames()) do
        local info = usableMonitor(name)

        if info then
            out[#out + 1] = info
        end
    end

    table.sort(out, function(a, b)
        if a.area ~= b.area then
            return a.area > b.area
        end

        return a.name < b.name
    end)

    return out
end

function display_alloc.find(shared, role)
    local mon, name = configured(shared, role)

    if mon then
        return mon, name
    end

    local monitors = listMonitors()
    local displays = shared.displays or {}
    local otherName

    if #monitors == 0 then
        return nil
    end

    if role == "main" then
        otherName = normalizeName(displays.attitude)
    elseif role == "attitude" then
        otherName = normalizeName(displays.main)
    end

    if role == "attitude" and #monitors >= 2 then
        for i = #monitors, 1, -1 do
            local info = monitors[i]

            if info.name ~= otherName then
                return info.mon, info.name
            end
        end
    end

    for _, info in ipairs(monitors) do
        if info.name ~= otherName then
            return info.mon, info.name
        end
    end

    local fallback = monitors[1]
    return fallback.mon, fallback.name
end

return display_alloc
