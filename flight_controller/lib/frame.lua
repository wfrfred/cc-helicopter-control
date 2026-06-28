local mathx = require("lib.mathx")

--- Coordinate frame with origin and orientation expressed in world coordinates.
---
--- Contract:
--- - `qWorldFromLocal` is the local-frame-to-world-frame orientation.
--- - Local vector coordinates are CC `vector` values. For FRD frames, local
---   x/y/z mean forward/right/down.
--- - `Frame` accepts and returns only CC `vector` and `quaternion` values.
---   Flight-control FRD plain tables are adapted in `lib.frames`.
--- - `componentsOf` and `vector` are for free vectors.
--- - `coordinatesOf` and `point` are for points and use `origin`.
--- - `localOrientationOf` and `worldOrientation` compose object orientations.
local frame = {}

---@class FrameBasis
---@field forward vector
---@field right vector
---@field down vector

---@class Frame
---@field origin vector
---@field qWorldFromLocal quaternion
local Frame = {}
Frame.__index = Frame

---@param q quaternion
---@return quaternion
local function shortest(q)
    if q.a >= 0.0 then
        return q
    end

    return -q
end

---@param q quaternion
---@param value vector
---@return vector
local function rotateVector(q, value)
    local t = q.v:cross(value) * 2.0

    return value + t * q.a + q.v:cross(t)
end

---@param origin vector|nil
---@param qWorldFromLocal quaternion|nil
---@return Frame
function frame.new(origin, qWorldFromLocal)
    return setmetatable({
        origin = origin or vector.new(0.0, 0.0, 0.0),
        qWorldFromLocal = (qWorldFromLocal or quaternion.identity()):normalize(),
    }, Frame)
end

---@return Frame
function frame.identity()
    return frame.new()
end

---@param qWorldFromLocal quaternion
---@param origin vector|nil
---@return Frame
function frame.fromQuaternion(qWorldFromLocal, origin)
    return frame.new(origin, qWorldFromLocal)
end

---@return FrameBasis
function Frame:basis()
    local q = self.qWorldFromLocal

    return {
        forward = rotateVector(q, vector.new(1.0, 0.0, 0.0)),
        right = rotateVector(q, vector.new(0.0, 1.0, 0.0)),
        down = rotateVector(q, vector.new(0.0, 0.0, 1.0)),
    }
end

---@param worldVector vector
---@return vector
function Frame:componentsOf(worldVector)
    return rotateVector(self.qWorldFromLocal:conjugate(), worldVector)
end

---@param localComponents vector
---@return vector
function Frame:vector(localComponents)
    return rotateVector(self.qWorldFromLocal, localComponents)
end

---@param worldPoint vector
---@return vector
function Frame:coordinatesOf(worldPoint)
    return self:componentsOf(worldPoint - self.origin)
end

---@param localCoordinates vector
---@return vector
function Frame:point(localCoordinates)
    return self.origin + self:vector(localCoordinates)
end

---@param qWorldFromObject quaternion
---@return quaternion
function Frame:localOrientationOf(qWorldFromObject)
    return shortest(self.qWorldFromLocal:conjugate() * qWorldFromObject):normalize()
end

---@param qLocalFromObject quaternion
---@return quaternion
function Frame:worldOrientation(qLocalFromObject)
    return (self.qWorldFromLocal * qLocalFromObject):normalize()
end

---@param qWorldFromObject quaternion
---@return vector
function Frame:rotationVectorTo(qWorldFromObject)
    local q = self:localOrientationOf(qWorldFromObject)
    local length = q.v:length()
    local scale = 2.0

    if length > 1.0e-9 then
        scale = 2.0 * mathx.atan2(length, q.a) / length
    end

    return q.v * scale
end

return frame
