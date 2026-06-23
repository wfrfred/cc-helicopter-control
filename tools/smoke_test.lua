local env = require("tools.test_env")
env.install()

local config = require("config")
local Controller = require("control.controller")
local attitude_math = require("lib.attitude_math")
local horizontal = require("control.horizontal")
local lock = require("modes.lock")
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
local command = controller:update({
    state = {
        raw = {},
        world = {
            position = vector.new(0.0, 80.0, 0.0),
            velocity = vector.new(0.0, 0.0, 0.0),
        },
        body = {
            frame = frame,
            orientation = orientation,
            pose = {
                roll = 0.0,
                pitch = 0.0,
                heading = 0.0,
                height = 80.0,
            },
            angular = {
                velocity = {
                    roll = 0.0,
                    pitch = 0.0,
                    yaw = 0.0,
                },
            },
        },
        navigation = {
            heading = {
                angle = 0.0,
                rate = 0.0,
            },
            velocity = {
                forward = 0.0,
                right = 0.0,
                up = 0.0,
            },
        },
        time = {
            pose = 0.0,
            velocity = 0.0,
            angularVelocity = 0.0,
        },
    },
    target = {
        source = "manual",
        attitude = {
            roll = 0.0,
            pitch = 0.0,
            feedforward = {
                angle = {
                    roll = 0.0,
                    pitch = 0.0,
                    yaw = 0.0,
                },
                rate = {
                    roll = 0.0,
                    pitch = 0.0,
                    yaw = 0.0,
                },
            },
        },
        world = {
            position = nil,
            velocity = nil,
            acceleration = nil,
        },
        vertical = {
            height = 80.0,
            speed = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = "locked",
        },
        heading = {
            angle = 0.0,
            rate = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = "locked",
        },
    },
    reset = {
        horizontal = false,
    },
    dt = config.control.loop.dt,
})
local controlTerms = controller:terms()
local oldYawPriority = "yaw" .. "_priority"
local oldYawPriorityTerm = "yaw" .. "Priority"
local oldReducedOrientation = "reduced" .. "Orientation"
local oldReducedFrameHelper = "reduced" .. "FrameFromTargetDown"

assert(config.control.heading[oldYawPriority] == nil, "heading yaw-priority config should be removed")
assert(controlTerms.attitude.target[oldYawPriorityTerm] == nil, "attitude target should not expose yaw-priority")
assert(controlTerms.attitude.target[oldReducedOrientation] == nil, "attitude target should not expose reduced orientation")
assert(attitude_math[oldReducedFrameHelper] == nil, "reduced target frame helper should be removed")

assertClose("neutral collective", command.collective, 1.0)
assertClose("neutral roll", command.roll, -0.0467649)
assertClose("neutral pitch", command.pitch, -0.33031404)
assertClose("neutral yaw", command.yaw, 0.00813396)

local hold = horizontal.new(config.control)
local holdResult = hold:updatePosition(
    vector.new(0.0, 80.0, 0.0),
    vector.new(0.0, 80.0, 0.0),
    vector.new(0.0, 0.0, 0.0),
    0.0,
    config.control.loop.dt
)
assert(holdResult.active == true, "position_hold should produce an active result")
assertClose("position_hold roll", holdResult.output.attitude.roll, 0.0)
assertClose("position_hold pitch", holdResult.output.attitude.pitch, 0.0)

local heightLock = lock.new({
    initial = 80.0,
    target_rate = config.control.vertical.target_rate,
    rate_deadband = config.control.vertical.lock.speed_deadband,
})
local lockResult = heightLock:update({
    input = 1.0,
    value = 80.0,
    rate = 0.0,
    dt = config.control.loop.dt,
})
assert(lockResult.active == false, "manual climb should disable height lock feedback")
assertClose("manual climb commanded rate", lockResult.rate, config.control.vertical.target_rate)

local navigator = navigation.new(config.navigation)
local navResult = navigator:command(
    { action = "activate", waypoint = "home" },
    {
        world = {
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
