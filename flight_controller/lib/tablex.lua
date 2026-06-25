local tablex = {}

function tablex.copy(value)
    local out = {}

    for key, item in pairs(value) do
        out[key] = item
    end

    return out
end

function tablex.deepCopy(value, seen)
    if type(value) ~= "table" then
        return value
    end

    seen = seen or {}

    if seen[value] ~= nil then
        return seen[value]
    end

    local out = {}
    seen[value] = out

    for key, item in pairs(value) do
        out[tablex.deepCopy(key, seen)] = tablex.deepCopy(item, seen)
    end

    return out
end

function tablex.keys(value)
    local out = {}

    for key in pairs(value) do
        out[#out + 1] = key
    end

    return out
end

function tablex.values(value)
    local out = {}

    for _, item in pairs(value) do
        out[#out + 1] = item
    end

    return out
end

function tablex.entries(value)
    local out = {}

    for key, item in pairs(value) do
        out[#out + 1] = {
            key = key,
            value = item,
        }
    end

    return out
end

function tablex.map(value, fn)
    local out = {}

    for index, item in ipairs(value) do
        out[index] = fn(item, index)
    end

    return out
end

function tablex.filter(value, fn)
    local out = {}

    for index, item in ipairs(value) do
        if fn(item, index) then
            out[#out + 1] = item
        end
    end

    return out
end

function tablex.reduce(value, fn, initial)
    local acc = initial
    local haveAcc = initial ~= nil

    for index, item in ipairs(value) do
        if haveAcc then
            acc = fn(acc, item, index)
        else
            acc = item
            haveAcc = true
        end
    end

    return acc
end

function tablex.merge(...)
    local out = {}

    for index = 1, select("#", ...) do
        local value = select(index, ...)

        if value ~= nil then
            for key, item in pairs(value) do
                out[key] = item
            end
        end
    end

    return out
end

function tablex.pick(value, keys)
    local out = {}

    for _, key in ipairs(keys) do
        out[key] = value[key]
    end

    return out
end

-- Turns column-oriented fields into row-oriented records.
-- Example:
-- transpose({ "height" }, { target = { height = 10 }, error = { height = 2 } })
-- returns { height = { target = 10, error = 2 } }.
function tablex.transpose(keys, columns)
    local out = {}

    for _, key in ipairs(keys) do
        local row = {}

        for name, column in pairs(columns) do
            row[name] = column[key]
        end

        out[key] = row
    end

    return out
end

function tablex.untranspose(keys, rows)
    local out = {}

    for _, key in ipairs(keys) do
        local row = rows[key]

        if row ~= nil then
            for name, item in pairs(row) do
                out[name] = out[name] or {}
                out[name][key] = item
            end
        end
    end

    return out
end

return tablex
