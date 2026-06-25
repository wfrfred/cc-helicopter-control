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

function mathx.affine(x, gain, bias)
    return (bias or 0.0) + (gain or 0.0) * x
end

function mathx.directionalAffine(x, gainNeg, gainPos, biasNeg, biasPos)
    if x < 0.0 then
        return mathx.affine(x, gainNeg, biasNeg)
    end

    if x > 0.0 then
        return mathx.affine(x, gainPos, biasPos)
    end

    return 0.0
end

function mathx.signNonZero(x)
    if x < 0.0 then
        return -1.0
    end

    return 1.0
end

function mathx.vectorLength(v)
    if type(v.length) == "function" then
        return v:length()
    end

    return #v
end

function mathx.normalizeVector(v)
    local length = mathx.vectorLength(v)

    assert(length > 0.0, "vector length must be positive")

    if type(v.normalize) == "function" then
        return v:normalize()
    end

    return v / length
end

function mathx.safeNormalizeVector(v, fallback)
    if mathx.vectorLength(v) > 1.0e-6 then
        return mathx.normalizeVector(v)
    end

    assert(fallback ~= nil, "safe normalize fallback must be set")

    return mathx.normalizeVector(fallback)
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
