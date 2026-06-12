local mathx = {}

function mathx.atan2(y, x)
    if math.atan2 then
        return math.atan2(y, x)
    end

    return math.atan(y, x)
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

return mathx
