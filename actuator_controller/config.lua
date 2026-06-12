local config = {}

config.modem = {
    side = "top",
}

config.sync = {
    enabled = false,
    sources = { "actuator_controller" },
}

config.actuator = {
    listen = "upper",
    display_dt = 0.5,
    outputs = {
        {
            blade = 1,
            polarity = -1,
            side = "back",
        },
        {
            blade = 2,
            polarity = 1,
            side = "left",
        },
    },
}

return config
