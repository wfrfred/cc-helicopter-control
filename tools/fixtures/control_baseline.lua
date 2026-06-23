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
                height = 80.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.0467649,
                pitch = -0.33031404,
                yaw = 0.00813396,
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
                height = 80.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = 0.47162460124153138,
                pitch = -0.35086139761285362,
                yaw = -0.16256342784519828,
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
                height = 80.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.020840094344622764,
                pitch = 0.18513728742206212,
                yaw = 0.010893457373866773,
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
                height = 80.0,
                verticalSpeed = 5.0,
                heightActive = false,
            },
            command = {
                collective = 6.0,
                roll = -0.0467649,
                pitch = -0.33031404,
                yaw = 0.00813396,
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
                height = 80.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.094812900000000006,
                pitch = -0.20862604000000001,
                yaw = 8.0,
            },
        },
    },
    {
        name = "position_hold neutral",
        expected = {
            mode = "position_hold",
            horizontalActive = true,
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
                height = 100.0,
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
