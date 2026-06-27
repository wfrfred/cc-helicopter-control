local frame = require("lib.frame")

--- Flight-control constructors and FRD adapters for `lib.frame`.
---
--- FRD tables are plain `{ forward, right, down }`; `Frame` methods use vector
--- coordinates where x/y/z mean forward/right/down in the local frame.
local frames = {}

local function shortest(q)
    if q.a >= 0.0 then
        return q
    end

    return -q
end

local function quaternionFromBasis(basis)
    local rotation = matrix.from2DArray({
        { basis.forward.x, basis.right.x, basis.down.x },
        { basis.forward.y, basis.right.y, basis.down.y },
        { basis.forward.z, basis.right.z, basis.down.z },
    })

    return shortest(quaternion.fromMatrix(rotation):normalize())
end

function frames.vectorFromFrd(frd)
    return vector.new(frd.forward or 0.0, frd.right or 0.0, frd.down or 0.0)
end

function frames.frdFromVector(value)
    return {
        forward = value.x,
        right = value.y,
        down = value.z,
    }
end

function frames.world(origin)
    return frame.new(origin, quaternion.identity())
end

function frames.fromBasis(basis, origin)
    return frame.new(origin, quaternionFromBasis(basis))
end

function frames.level(heading)
    return frames.levelAt(nil, heading)
end

function frames.levelAt(origin, heading)
    return frames.fromBasis({
        forward = vector.new(math.sin(heading), 0.0, -math.cos(heading)),
        right = vector.new(math.cos(heading), 0.0, math.sin(heading)),
        down = vector.new(0.0, -1.0, 0.0),
    }, origin)
end

function frames.bodyFromAngles(roll, pitch, heading, origin)
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

    return frames.fromBasis({
        forward = forward,
        right = rightLevel * cosRoll + downLevel * sinRoll,
        down = rightLevel * -sinRoll + downLevel * cosRoll,
    }, origin)
end

function frames.bodyFromBasis(basis, origin)
    return frames.fromBasis(basis, origin)
end

function frames.bodyFromPose(rawPose, bodyAxis)
    local q = rawPose.orientation:normalize()

    return frames.bodyFromBasis({
        forward = q:mul(bodyAxis.forward),
        right = q:mul(bodyAxis.right),
        down = q:mul(bodyAxis.down),
    }, rawPose.position)
end

return frames
