local M = {}

local root = debug.getinfo(1, "S").source:sub(2):match("^(.*)/tools/test_env%.lua$")

if root == "" then
    root = "."
end

local function prependPackagePath(path)
    package.path = root .. "/" .. path .. "/?.lua;"
        .. root .. "/" .. path .. "/?/init.lua;"
        .. package.path
end

function M.installPaths()
    prependPackagePath("actuator_controller")
    prependPackagePath("user_interface")
    prependPackagePath("flight_controller")
end

local vectorMethods = {}
local vectorMetatable = {
    __name = "vector",
    __index = vectorMethods,
}

local function newVector(x, y, z)
    return setmetatable({
        x = tonumber(x) or 0.0,
        y = tonumber(y) or 0.0,
        z = tonumber(z) or 0.0,
    }, vectorMetatable)
end

function vectorMethods:add(other)
    return newVector(self.x + other.x, self.y + other.y, self.z + other.z)
end

function vectorMethods:sub(other)
    return newVector(self.x - other.x, self.y - other.y, self.z - other.z)
end

function vectorMethods:mul(value)
    return newVector(self.x * value, self.y * value, self.z * value)
end

function vectorMethods:div(value)
    return newVector(self.x / value, self.y / value, self.z / value)
end

function vectorMethods:unm()
    return newVector(-self.x, -self.y, -self.z)
end

function vectorMethods:dot(other)
    return self.x * other.x + self.y * other.y + self.z * other.z
end

function vectorMethods:cross(other)
    return newVector(
        self.y * other.z - self.z * other.y,
        self.z * other.x - self.x * other.z,
        self.x * other.y - self.y * other.x
    )
end

function vectorMethods:length()
    return math.sqrt(self:dot(self))
end

function vectorMethods:normalize()
    return self / self:length()
end

vectorMetatable.__add = vectorMethods.add
vectorMetatable.__sub = vectorMethods.sub
vectorMetatable.__mul = vectorMethods.mul
vectorMetatable.__div = vectorMethods.div
vectorMetatable.__unm = vectorMethods.unm

local quaternionMethods = {}
local quaternionMetatable = {
    __name = "quaternion",
    __index = quaternionMethods,
}

local function newQuaternion(vec, scalar)
    return setmetatable({
        v = vec or newVector(0.0, 0.0, 0.0),
        a = scalar == nil and 1.0 or scalar,
    }, quaternionMetatable)
end

local function fromComponents(x, y, z, w)
    return newQuaternion(newVector(x, y, z), w)
end

local function fromAxisAngle(axis, angle)
    local normalized = axis:normalize()
    local half = angle * 0.5

    return newQuaternion(normalized * math.sin(half), math.cos(half))
end

function quaternionMethods:normalize()
    local length = math.sqrt(
        self.a * self.a
            + self.v.x * self.v.x
            + self.v.y * self.v.y
            + self.v.z * self.v.z
    )

    return newQuaternion(self.v / length, self.a / length)
end

function quaternionMethods:conjugate()
    return newQuaternion(-self.v, self.a)
end

function quaternionMethods:slerp(target, t)
    local q1 = self:normalize()
    local q2 = target:normalize()
    local dot = q1.a * q2.a + q1.v:dot(q2.v)

    if dot < 0.0 then
        q2 = -q2
        dot = -dot
    end

    if dot > 0.9995 then
        return newQuaternion(q1.v * (1.0 - t) + q2.v * t, q1.a * (1.0 - t) + q2.a * t):normalize()
    end

    local theta0 = math.acos(dot)
    local theta = theta0 * t
    local sinTheta = math.sin(theta)
    local sinTheta0 = math.sin(theta0)
    local s0 = math.cos(theta) - dot * sinTheta / sinTheta0
    local s1 = sinTheta / sinTheta0

    return newQuaternion(q1.v * s0 + q2.v * s1, q1.a * s0 + q2.a * s1)
end

local function rotateVector(q, value)
    local p = newQuaternion(value, 0.0)
    local rotated = q * p * q:conjugate()

    return rotated.v
end

function quaternionMethods:mul(other)
    if getmetatable(other) == vectorMetatable then
        return rotateVector(self, other)
    end

    return newQuaternion(
        self.v * other.a + other.v * self.a + self.v:cross(other.v),
        self.a * other.a - self.v:dot(other.v)
    )
end

function quaternionMethods:unm()
    return newQuaternion(-self.v, -self.a)
end

quaternionMetatable.__mul = quaternionMethods.mul
quaternionMetatable.__unm = quaternionMethods.unm

function M.installRuntimeGlobals()
    _G.colors = _G.colors or {
        white = 1,
        orange = 2,
        magenta = 4,
        lightBlue = 8,
        yellow = 16,
        lime = 32,
        pink = 64,
        gray = 128,
        lightGray = 256,
        cyan = 512,
        purple = 1024,
        blue = 2048,
        brown = 4096,
        green = 8192,
        red = 16384,
        black = 32768,
        toBlit = function()
            return "0"
        end,
    }

    _G.vector = {
        new = newVector,
    }

    _G.quaternion = {
        new = newQuaternion,
        fromComponents = fromComponents,
        fromAxisAngle = fromAxisAngle,
        identity = function()
            return newQuaternion()
        end,
    }

    _G.sleep = _G.sleep or function() end
    _G.rednet = _G.rednet or {
        open = function() end,
        broadcast = function() end,
        receive = function()
            return nil
        end,
    }
    _G.peripheral = _G.peripheral or {
        wrap = function()
            return nil
        end,
    }
    _G.parallel = _G.parallel or {
        waitForAny = function(...)
            local tasks = { ... }
            return tasks[1]()
        end,
    }
end

function M.install()
    M.installPaths()
    M.installRuntimeGlobals()
end

return M
