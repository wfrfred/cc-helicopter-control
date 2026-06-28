local frame = require("lib.frame")
local mathx = require("lib.mathx")

--- Flight-control constructors and FRD adapters for `lib.frame`.
---
--- FRD tables are plain `{ forward, right, down }`; `Frame` methods use vector
--- coordinates where x/y/z mean forward/right/down in the local frame.
--- `body` and `bodyAngularVector` are SableCC API-boundary adapters; the rest
--- of the system consumes semantic body/world/nav values.
local frames = {}

---@class FrdVector
---@field forward number
---@field right number
---@field down number

---@class FrdVectorInput
---@field forward number|nil
---@field right number|nil
---@field down number|nil

---@class BodyAxis
---@field forward vector
---@field right vector
---@field down vector

---@class RawPose
---@field position vector
---@field orientation quaternion

---@param q quaternion
---@return quaternion
local function shortest(q)
    if q.a >= 0.0 then
        return q
    end

    return -q
end

---@param basis FrameBasis
---@return quaternion
local function quaternionFromBasis(basis)
    local rotation = matrix.from2DArray({
        { basis.forward.x, basis.right.x, basis.down.x },
        { basis.forward.y, basis.right.y, basis.down.y },
        { basis.forward.z, basis.right.z, basis.down.z },
    })

    return shortest(quaternion.fromMatrix(rotation):normalize())
end

---@param value vector
---@param axis vector
---@return number
local function component(value, axis)
    return value.x * axis.x + value.y * axis.y + value.z * axis.z
end

---@param frd FrdVectorInput
---@return vector
function frames.vectorFromFrd(frd)
    return vector.new(frd.forward or 0.0, frd.right or 0.0, frd.down or 0.0)
end

---@param value vector
---@return FrdVector
function frames.frdFromVector(value)
    return {
        forward = value.x,
        right = value.y,
        down = value.z,
    }
end

---@param origin vector|nil
---@return Frame
function frames.world(origin)
    return frame.new(origin, quaternion.identity())
end

---@param basis FrameBasis
---@param origin vector|nil
---@return Frame
function frames.fromBasis(basis, origin)
    return frame.new(origin, quaternionFromBasis(basis))
end

---@param heading number
---@return Frame
function frames.level(heading)
    return frames.levelAt(nil, heading)
end

---@param origin vector|nil
---@param heading number
---@return Frame
function frames.levelAt(origin, heading)
    return frames.fromBasis({
        forward = vector.new(math.sin(heading), 0.0, -math.cos(heading)),
        right = vector.new(math.cos(heading), 0.0, math.sin(heading)),
        down = vector.new(0.0, -1.0, 0.0),
    }, origin)
end

---@param roll number
---@param pitch number
---@param heading number
---@param origin vector|nil
---@return Frame
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

---@param basis FrameBasis
---@param origin vector|nil
---@return Frame
function frames.bodyFromBasis(basis, origin)
    return frames.fromBasis(basis, origin)
end

---@param rawPose RawPose
---@param bodyAxis BodyAxis
---@return Frame
function frames.body(rawPose, bodyAxis)
    local q = rawPose.orientation:normalize()
    local rawFrame = frame.fromQuaternion(q, rawPose.position)

    return frames.bodyFromBasis({
        forward = rawFrame:vector(bodyAxis.forward),
        right = rawFrame:vector(bodyAxis.right),
        down = rawFrame:vector(bodyAxis.down),
    }, rawPose.position)
end

---@param bodyFrame Frame
---@return Frame
function frames.navigation(bodyFrame)
    local basis = bodyFrame:basis()
    local heading = mathx.atan2(basis.forward.x, -basis.forward.z)

    return frames.levelAt(bodyFrame.origin, heading)
end

---@param rawAngularVelocity vector
---@param bodyAxis BodyAxis
---@return vector
function frames.bodyAngularVector(rawAngularVelocity, bodyAxis)
    return vector.new(
        component(rawAngularVelocity, bodyAxis.forward),
        component(rawAngularVelocity, bodyAxis.right),
        component(rawAngularVelocity, bodyAxis.down)
    )
end

return frames
