local M = {}

local cases = {
    {
        name = "manual neutral",
        input = {
            roll = 0.0,
            pitch = 0.0,
            climb = 0.0,
            heading = 0.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.0,
                pitch = 0.0,
                height = 0.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.03,
                pitch = -0.33,
                yaw = 0.0,
            },
        },
    },
    {
        name = "manual roll positive",
        input = {
            roll = 1.0,
            pitch = 0.0,
            climb = 0.0,
            heading = 0.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.039269908169872414,
                pitch = 0.0,
                height = 0.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = 0.486810644722,
                pitch = -0.33,
                yaw = 0.0,
            },
        },
    },
    {
        name = "manual pitch positive",
        input = {
            roll = 0.0,
            pitch = 1.0,
            climb = 0.0,
            heading = 0.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.0,
                pitch = 0.039269908169872414,
                height = 0.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.03,
                pitch = 0.183108474129,
                yaw = 0.0,
            },
        },
    },
    {
        name = "manual climb positive",
        input = {
            roll = 0.0,
            pitch = 0.0,
            climb = 1.0,
            heading = 0.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.0,
                pitch = 0.0,
                verticalSpeed = 5.0,
                heightActive = false,
            },
            command = {
                collective = 6.0,
                roll = -0.03,
                pitch = -0.33,
                yaw = 0.0,
            },
        },
    },
    {
        name = "manual heading positive",
        input = {
            roll = 0.0,
            pitch = 0.0,
            climb = 0.0,
            heading = 1.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.0,
                pitch = 0.0,
                height = 0.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.03,
                pitch = -0.33,
                yaw = 8.0,
            },
        },
    },
    {
        name = "position_hold neutral",
        expected = {
            mode = "position_hold",
            horizontalKind = "position",
            target = {
                roll = 0.0,
                pitch = 0.0,
            },
        },
    },
    {
        name = "cruise capture",
        expected = {
            mode = "cruise",
            cruise = {
                x = 3.0,
                z = -1.0,
                height = 0.0,
                heading = 0.0,
            },
            target = {
                roll = 0.052500000000000005,
                pitch = -0.0155,
            },
        },
    },
    {
        name = "navigation active target",
        expected = {
            mode = "navigation",
            active = true,
            phase = "climb",
            target = {
                x = -213.0,
                z = 304.0,
                height = 120.0,
                heading = 0.0,
            },
        },
    },
    {
        name = "input stale zero input behavior",
        inputAge = 0.6,
        expected = {
            stale = true,
            manualAxes = {
                roll = 0.0,
                pitch = 0.0,
                climb = 0.0,
                heading = 0.0,
            },
        },
    },
}

function M.cases()
    return cases
end

return M
