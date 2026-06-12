local quat = {}

function quat.new(x, y, z, w)
    return {
        x = x,
        y = y,
        z = z,
        w = w,
    }
end

function quat.fromSable(q)
    return quat.new(q.v.x, q.v.y, q.v.z, q.a)
end

function quat.normalize(q)
    local n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)

    assert(n > 0, "zero quaternion")

    return quat.new(q.x / n, q.y / n, q.z / n, q.w / n)
end

function quat.conjugate(q)
    return quat.new(-q.x, -q.y, -q.z, q.w)
end

function quat.inverse(q)
    return quat.conjugate(quat.normalize(q))
end

function quat.mul(a, b)
    return quat.new(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    )
end

function quat.rotate(q, v)
    q = quat.normalize(q)

    local p = quat.new(v.x, v.y, v.z, 0)
    local r = quat.mul(quat.mul(q, p), quat.inverse(q))

    return vector.new(r.x, r.y, r.z)
end

return quat
