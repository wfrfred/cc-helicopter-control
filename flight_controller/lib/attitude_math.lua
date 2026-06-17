local attitude_math = {}

local function atan2(y, x)
    if math.atan2 ~= nil then
        return math.atan2(y, x)
    end

    return math.atan(y, x)
end

local function normalizeQuaternion(q)
    local norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)

    assert(norm > 0.0, "attitude quaternion norm must be positive")

    return {
        w = q.w / norm,
        x = q.x / norm,
        y = q.y / norm,
        z = q.z / norm,
    }
end

local function conjugate(q)
    return {
        w = q.w,
        x = -q.x,
        y = -q.y,
        z = -q.z,
    }
end

local function multiply(a, b)
    return {
        w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    }
end

local function shortest(q)
    if q.w >= 0.0 then
        return q
    end

    return {
        w = -q.w,
        x = -q.x,
        y = -q.y,
        z = -q.z,
    }
end

function attitude_math.frameFromPose(roll, pitch, heading)
    local sinHeading = math.sin(heading)
    local cosHeading = math.cos(heading)
    local sinPitch = math.sin(pitch)
    local cosPitch = math.cos(pitch)
    local sinRoll = math.sin(roll)
    local cosRoll = math.cos(roll)

    local forwardHorizontal = vector.new(sinHeading, 0.0, -cosHeading)
    local rightLevel = vector.new(cosHeading, 0.0, sinHeading)
    local worldDown = vector.new(0.0, -1.0, 0.0)
    local forward = forwardHorizontal * cosPitch + worldDown * -sinPitch
    local downLevel = forward:cross(rightLevel)

    return {
        forward = forward,
        right = rightLevel * cosRoll + downLevel * sinRoll,
        down = rightLevel * -sinRoll + downLevel * cosRoll,
    }
end

function attitude_math.quaternionFromFrame(frame)
    assert(type(frame) == "table", "attitude frame must be table")

    local m00 = frame.forward.x
    local m01 = frame.right.x
    local m02 = frame.down.x
    local m10 = frame.forward.y
    local m11 = frame.right.y
    local m12 = frame.down.y
    local m20 = frame.forward.z
    local m21 = frame.right.z
    local m22 = frame.down.z
    local trace = m00 + m11 + m22
    local q

    if trace > 0.0 then
        local s = math.sqrt(trace + 1.0) * 2.0
        q = {
            w = 0.25 * s,
            x = (m21 - m12) / s,
            y = (m02 - m20) / s,
            z = (m10 - m01) / s,
        }
    elseif m00 > m11 and m00 > m22 then
        local s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = {
            w = (m21 - m12) / s,
            x = 0.25 * s,
            y = (m01 + m10) / s,
            z = (m02 + m20) / s,
        }
    elseif m11 > m22 then
        local s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = {
            w = (m02 - m20) / s,
            x = (m01 + m10) / s,
            y = 0.25 * s,
            z = (m12 + m21) / s,
        }
    else
        local s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = {
            w = (m10 - m01) / s,
            x = (m02 + m20) / s,
            y = (m12 + m21) / s,
            z = 0.25 * s,
        }
    end

    return shortest(normalizeQuaternion(q))
end

function attitude_math.relativeQuaternion(current, target)
    assert(type(current) == "table", "current attitude quaternion must be table")
    assert(type(target) == "table", "target attitude quaternion must be table")

    return shortest(normalizeQuaternion(multiply(conjugate(current), target)))
end

function attitude_math.attitudeError(current, target)
    local q = attitude_math.relativeQuaternion(current, target)
    local vectorNorm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z)
    local scale = 2.0

    if vectorNorm > 1.0e-9 then
        scale = 2.0 * atan2(vectorNorm, q.w) / vectorNorm
    end

    return {
        roll = q.x * scale,
        pitch = q.y * scale,
        yaw = q.z * scale,
    }
end

return attitude_math
