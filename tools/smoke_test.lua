local env = require("tools.test_env")
env.install()

local config = require("config")
local Controller = require("controller")
local attitude_math = require("lib.attitude_math")
local position_hold = require("position_hold")
local rate_lock = require("rate_lock")
local navigation = require("navigation")

local function assertClose(name, actual, expected, tolerance)
    tolerance = tolerance or 1.0e-9
    assert(
        math.abs(actual - expected) <= tolerance,
        string.format("%s expected %.12f got %.12f", name, expected, actual)
    )
end

local frame = attitude_math.frameFromPose(0.0, 0.0, 0.0)
local orientation = attitude_math.quaternionFromFrame(frame)
local controller = Controller.new(config.control)
local result = controller:update({
    state = {
        bodyFrame = frame,
        orientation = orientation,
        pose = {
            roll = 0.0,
            pitch = 0.0,
            heading = 0.0,
            height = 80.0,
        },
        rates = {
            roll = 0.0,
            pitch = 0.0,
            yaw = 0.0,
        },
        vertical = {
            height = 80.0,
            speed = 0.0,
        },
    },
    target = {
        attitude = {
            roll = 0.0,
            pitch = 0.0,
            source = "smoke",
            orientation = orientation,
            fullOrientation = orientation,
            reducedOrientation = orientation,
            yawPriority = 1.0,
        },
        vertical = {
            height = 80.0,
            speed = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = "locked",
        },
    },
    dt = config.control.loop.dt,
})

assertClose("neutral collective", result.commands.collective, 1.0)
assertClose("neutral roll", result.commands.roll, -0.03)
assertClose("neutral pitch", result.commands.pitch, -0.33)
assertClose("neutral yaw", result.commands.yaw, 0.0)

local hold = position_hold.new(config.control)
local holdResult = hold:update({ x = 0.0, z = 0.0 }, { x = 0.0, z = 0.0 }, 0.0, config.control.loop.dt)
assert(holdResult.active == true, "position_hold should produce an active result")
assertClose("position_hold roll", holdResult.output.attitude.roll, 0.0)
assertClose("position_hold pitch", holdResult.output.attitude.pitch, 0.0)

local lock = rate_lock.new({
    initial_target = 80.0,
    target_rate = config.control.vertical.target_rate,
    rate_deadband = config.control.vertical.lock.speed_deadband,
})
local lockResult = lock:update(1.0, 80.0, 0.0, config.control.loop.dt)
assert(lockResult.active == false, "manual climb should disable height lock feedback")
assertClose("manual climb commanded rate", lockResult.commandedRate, config.control.vertical.target_rate)

local navigator = navigation.new(config.navigation)
local navResult = navigator:command(
    { action = "activate", waypoint = "home" },
    {
        raw = {
            position = vector.new(-213, 90, 304),
        },
        body = {
            pose = {
                heading = 0.0,
            },
        },
    },
    {
        worldVelocity = { x = 0.0, z = 0.0 },
        verticalSpeed = 0.0,
        headingRate = 0.0,
    }
)
assert(navResult.active == true, "navigation should activate configured home waypoint")

print("smoke ok")
