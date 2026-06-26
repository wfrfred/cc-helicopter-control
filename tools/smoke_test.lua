local env = require("tools.test_env")
env.install()

local config = require("config")
local Controller = require("control.controller")
local attitude_math = require("lib.attitude_math")
local common = require("modes.common")
local horizontal = require("control.horizontal")
local lock = require("modes.lock")
local navigation = require("modes.navigation")

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
local controllerTarget = common.target("attitude")

controllerTarget.altitude.position = 0.0
controllerTarget.horizontal.angle.roll = 0.0
controllerTarget.horizontal.angle.pitch = 0.0
controllerTarget.yaw.angle = 0.0

local control = controller:update({
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
    target = controllerTarget,
    dt = config.control.loop.dt,
})
local command = control.output
local controlTerms = control.terms
local oldYawPriority = "yaw" .. "_priority"
local oldYawPriorityTerm = "yaw" .. "Priority"
local oldReducedOrientation = "reduced" .. "Orientation"
local oldReducedFrameHelper = "reduced" .. "FrameFromTargetDown"

assert(config.control.heading[oldYawPriority] == nil, "heading yaw-priority config should be removed")
assert(controlTerms.attitude.target == nil, "attitude terms should not expose split target wrapper")
assert(controlTerms.attitude[oldYawPriorityTerm] == nil, "attitude terms should not expose yaw-priority")
assert(controlTerms.attitude[oldReducedOrientation] == nil, "attitude terms should not expose reduced orientation")
assert(attitude_math[oldReducedFrameHelper] == nil, "reduced target frame helper should be removed")

assertClose("neutral collective", command.collective, 1.0)
assertClose("neutral roll", command.roll, -0.0467649)
assertClose("neutral pitch", command.pitch, -0.33031404)
assertClose("neutral yaw", command.yaw, 0.00813396)

local hold = horizontal.new(config.control)
local holdTarget = common.target("position").horizontal

holdTarget.position.forward = 0.0
holdTarget.position.right = 0.0

local holdResult = hold:update(
    {
        position = {
            forward = 0.0,
            right = 0.0,
        },
        velocity = {
            forward = 0.0,
            right = 0.0,
        },
    },
    {
        position = holdTarget.position,
    },
    holdTarget.feedforward,
    config.control.loop.dt
)
assertClose("position_hold roll", holdResult.output.angle.roll, 0.0)
assertClose("position_hold pitch", holdResult.output.angle.pitch, 0.0)

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

local navState = {
    world = {
        position = vector.new(-213, 90, 304),
        velocity = vector.new(0.0, 0.0, 0.0),
    },
    body = {
        pose = {
            heading = 0.0,
            height = 90.0,
        },
    },
    navigation = {
        heading = {
            angle = 0.0,
            rate = 0.0,
        },
    },
}
local navMode = navigation.new(config.navigation)
navMode:enter({
    command = { action = "activate", waypoint = "home" },
    state = navState,
    dt = config.control.loop.dt,
})
assert(navMode:terms(navState).active == true, "navigation should activate configured home waypoint")

print("smoke ok")
