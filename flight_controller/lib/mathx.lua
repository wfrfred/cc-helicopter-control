local mathx = {}

function mathx.atan2(y, x)
    return math.atan2(y, x)
end

function mathx.wrapPi(x)
    while x > math.pi do
        x = x - 2 * math.pi
    end

    while x < -math.pi do
        x = x + 2 * math.pi
    end

    return x
end

function mathx.clamp(x, lo, hi)
    if x < lo then
        return lo
    end

    if x > hi then
        return hi
    end

    return x
end

function mathx.component(value, axis)
    return (value.x or 0.0) * (axis.x or 0.0)
        + (value.y or 0.0) * (axis.y or 0.0)
        + (value.z or 0.0) * (axis.z or 0.0)
end

function mathx.project(value, axes)
    local out = {}

    for name, axis in pairs(axes) do
        out[name] = mathx.component(value, axis)
    end

    return out
end

return mathx
