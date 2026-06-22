local mathx = require("lib.mathx")

local attitude_math = {}

local function atan2(y, x)
    if math.atan2 ~= nil then
        return math.atan2(y, x)
    end

    return math.atan(y, x)
end

local function quaternionApi()
    assert(type(quaternion) == "table", "quaternion API must be loaded")
    assert(
        type(quaternion.fromMatrix) == "function",
        "quaternion.fromMatrix must be available"
    )

    return quaternion
end

local function matrixApi()
    assert(type(matrix) == "table", "matrix API must be loaded")
    assert(type(matrix.from2DArray) == "function", "matrix.from2DArray must be available")

    return matrix
end

local function shortest(q)
    if q.a >= 0.0 then
        return q
    end

    return -q
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

function attitude_math.reducedFrameFromTargetDown(currentFrame, fullTargetFrame)
    assert(type(currentFrame) == "table", "current attitude frame must be table")
    assert(type(fullTargetFrame) == "table", "target attitude frame must be table")

    local down = mathx.normalizeVector(fullTargetFrame.down)
    local forward = currentFrame.forward - down * currentFrame.forward:dot(down)

    if mathx.vectorLength(forward) < 1.0e-6 then
        forward = currentFrame.right:cross(down)
    end

    forward = mathx.normalizeVector(forward)

    local right = mathx.normalizeVector(down:cross(forward))

    return {
        forward = forward,
        right = right,
        down = down,
    }
end

function attitude_math.quaternionFromFrame(frame)
    assert(type(frame) == "table", "attitude frame must be table")

    local rotation = matrixApi().from2DArray({
        { frame.forward.x, frame.right.x, frame.down.x },
        { frame.forward.y, frame.right.y, frame.down.y },
        { frame.forward.z, frame.right.z, frame.down.z },
    })

    return shortest(quaternionApi().fromMatrix(rotation):normalize())
end

function attitude_math.relativeQuaternion(current, target)
    assert(type(current) == "table", "current attitude quaternion must be table")
    assert(type(target) == "table", "target attitude quaternion must be table")

    local qCurrent = current:normalize()
    local qTarget = target:normalize()

    return shortest(qCurrent:conjugate() * qTarget):normalize()
end

function attitude_math.bodyRateCommand(current, target, tau)
    assert(type(current) == "table", "current attitude quaternion must be table")
    assert(type(target) == "table", "target attitude quaternion must be table")
    assert(type(tau) == "number" and tau > 0.0, "attitude time_constant must be positive")

    local qe = attitude_math.relativeQuaternion(current, target):normalize()
    local gain = 2.0 / tau
    local sign = mathx.signNonZero(qe.a)

    return {
        roll = gain * sign * qe.v.x,
        pitch = gain * sign * qe.v.y,
        yaw = gain * sign * qe.v.z,
    }
end

function attitude_math.attitudeError(current, target)
    local q = attitude_math.relativeQuaternion(current, target)
    local vectorNorm = mathx.vectorLength(q.v)
    local scale = 2.0

    if vectorNorm > 1.0e-9 then
        scale = 2.0 * atan2(vectorNorm, q.a) / vectorNorm
    end

    return {
        roll = q.v.x * scale,
        pitch = q.v.y * scale,
        yaw = q.v.z * scale,
    }
end

return attitude_math
