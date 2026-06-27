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

local Frame = {}
Frame.__index = Frame

local function shortest(q)
    if q.a >= 0.0 then
        return q
    end

    return -q
end

local function rotateVector(q, value)
    local t = q.v:cross(value) * 2.0

    return value + t * q.a + q.v:cross(t)
end

function frame.new(origin, qWorldFromLocal)
    return setmetatable({
        origin = origin or vector.new(0.0, 0.0, 0.0),
        qWorldFromLocal = (qWorldFromLocal or quaternion.identity()):normalize(),
    }, Frame)
end

function frame.identity()
    return frame.new()
end

function frame.fromQuaternion(qWorldFromLocal, origin)
    return frame.new(origin, qWorldFromLocal)
end

function Frame:basis()
    local q = self.qWorldFromLocal

    return {
        forward = rotateVector(q, vector.new(1.0, 0.0, 0.0)),
        right = rotateVector(q, vector.new(0.0, 1.0, 0.0)),
        down = rotateVector(q, vector.new(0.0, 0.0, 1.0)),
    }
end

function Frame:componentsOf(worldVector)
    return rotateVector(self.qWorldFromLocal:conjugate(), worldVector)
end

function Frame:vector(localComponents)
    return rotateVector(self.qWorldFromLocal, localComponents)
end

function Frame:coordinatesOf(worldPoint)
    return self:componentsOf(worldPoint - self.origin)
end

function Frame:point(localCoordinates)
    return self.origin + self:vector(localCoordinates)
end

function Frame:localOrientationOf(qWorldFromObject)
    return shortest(self.qWorldFromLocal:conjugate() * qWorldFromObject):normalize()
end

function Frame:worldOrientation(qLocalFromObject)
    return (self.qWorldFromLocal * qLocalFromObject):normalize()
end

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
