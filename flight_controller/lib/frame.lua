local mathx = require("lib.mathx")

--- Coordinate frame with origin and orientation expressed in world coordinates.
---
--- Vector and point methods take CC `vector` values. Flight-control FRD tables
--- are adapted in `lib.frames`, not accepted here implicitly.
local frame = {}

local Frame = {}
Frame.__index = Frame

local function shortest(q)
    if q.a >= 0.0 then
        return q
    end

    return -q
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
        forward = q:mul(vector.new(1.0, 0.0, 0.0)),
        right = q:mul(vector.new(0.0, 1.0, 0.0)),
        down = q:mul(vector.new(0.0, 0.0, 1.0)),
    }
end

function Frame:componentsOf(worldVector)
    return self.qWorldFromLocal:conjugate():mul(worldVector)
end

function Frame:vector(localComponents)
    return self.qWorldFromLocal:mul(localComponents)
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
