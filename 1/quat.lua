local quat = {}

function quat.new(x, y, z, w)
    return {
        x = x or 0,
        y = y or 0,
        z = z or 0,
        w = w or 1,
    }
end

function quat.fromSable(q)
    if q.a ~= nil and q.v ~= nil then
        return quat.new(q.v.x or 0, q.v.y or 0, q.v.z or 0, q.a or 1)
    end

    return quat.new(q.x or q[1] or 0, q.y or q[2] or 0, q.z or q[3] or 0, q.w or q[4] or 1)
end

function quat.normalize(q)
    local n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)

    if n <= 0 then
        return quat.new(0, 0, 0, 1)
    end

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

    local p = quat.new(v.x or v[1] or 0, v.y or v[2] or 0, v.z or v[3] or 0, 0)
    local r = quat.mul(quat.mul(q, p), quat.inverse(q))

    return vector.new(r.x, r.y, r.z)
end

return quat