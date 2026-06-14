local config = {}

config.sync = {
    enabled = false,
    sources = { "flight_controller" },
}

config.runtime = {
    modem = {
        control = "top",
    },

    input = {
        receive_timeout = 0.1,
        typewriter_name = "linked_typewriter",
    },

    telemetry = {
        broadcast_dt = 0.1,
    },

    data = {
        linear_velocity_dt = 0.1,
    },
}

config.hardware = {
    rotor = {
        modem_side = "front",
        upper_bearing = "swivel_bearing_4",
        lower_bearing = "swivel_bearing_5",
        blade_mount = {
            [1] = 0.0,
            [2] = math.pi / 2,
            [3] = math.pi,
            [4] = 3 * math.pi / 2,
        },
    },
}

config.calibration = {
    body_axis = {
        forward = vector.new(0, 0, 1),
        right = vector.new(-1, 0, 0),
        down = vector.new(0, -1, 0),
    },

    rotor = {
        phase_offset_upper = math.pi / 2,
        phase_offset_lower = math.pi / 2,
    },

    mixer_axis = {
        collective = 1,
        roll = -1,
        pitch = 1,
        yaw = -1,
    },
}

config.control = {
    loop = {
        dt = 0.05,
        max_dt = 0.2,
        telemetry_dt = 0.1,
    },

    input = {
        stale_dt = 0.5,
    },

    collective = {
        min = 0.0,
        max = 10.0,
        feedforward_bias = 1.0,
        tilt_compensation = {
            min_factor = 0.5,
        },
    },

    vertical = {
        target_rate = 5.0,
        speed_feedforward_gain = 0.5,
        lock = {
            speed_deadband = 0.1,
            relock_timeout = 0.6,
        },
    },

    attitude = {
        home = {
            roll = 0.0,
            pitch = 0.0,
        },

        limit = {
            roll = math.rad(30),
            pitch = math.rad(25),
        },

        target_rate = {
            roll = math.rad(45),
            pitch = math.rad(45),
        },

        center_rate = {
            roll = math.rad(60),
            pitch = math.rad(60),
        },

        pitch = {
            feedforward_bias = 0.3,
        },
    },

    yaw = {
        target_rate = math.rad(60),
        rate_feedforward_gain = 1.3 / math.rad(45),
        lock = {
            rate_deadband = math.rad(2),
            relock_timeout = 0.6,
        },
    },

    position_hold = {
        velocity_feedforward = {
            right = 0.016,
            forward = 0.018,
        },
    },

    pid = {
        vertical = {
            height = {
                kp = 1.0,
                ki = 0.5,
                kd = 0.0,
                i_min = -3.0,
                i_max = 3.0,
                out_min = -5.0,
                out_max = 5.0,
                deadband = 0.03,
            },

            speed = {
                kp = 0.5,
                ki = 0.0,
                kd = 0.0,
                i_min = -3.0,
                i_max = 3.0,
                out_min = -6.0,
                out_max = 6.0,
                deadband = 0.03,
            },
        },

        position = {
            right = {
                kp = 0.15,
                ki = 0.005,
                kd = 0.0,
                i_min = -10.0,
                i_max = 10.0,
                out_min = -20.0,
                out_max = 20.0,
                deadband = 0.05,
            },

            forward = {
                kp = 0.15,
                ki = 0.01,
                kd = 0.0,
                i_min = -10.0,
                i_max = 10.0,
                out_min = -20.0,
                out_max = 20.0,
                deadband = 0.05,
            },
        },

        velocity = {
            right = {
                kp = 0.030,
                ki = 0.01,
                kd = 0.005,
                i_min = -0.35,
                i_max = 0.35,
                out_min = -math.rad(20),
                out_max = math.rad(20),
                deadband = 0.05,
            },

            forward = {
                kp = 0.032,
                ki = 0.01,
                kd = 0.006,
                i_min = -0.35,
                i_max = 0.35,
                out_min = -math.rad(30),
                out_max = math.rad(30),
                deadband = 0.05,
            },
        },

        attitude = {
            roll = {
                kp = 6.0,
                ki = 0.0,
                kd = 0.5,
                i_min = -2.0,
                i_max = 2.0,
                out_min = -8.0,
                out_max = 8.0,
                deadband = math.rad(0.05),
            },

            pitch = {
                kp = 6.0,
                ki = 0.0,
                kd = 0.5,
                i_min = -1.5,
                i_max = 1.5,
                out_min = -12.0,
                out_max = 12.0,
                deadband = math.rad(0.15),
            },
        },

        yaw = {
            angle = {
                kp = 1.0,
                ki = 0.0,
                kd = 0.0,
                i_min = -0.5,
                i_max = 0.5,
                out_min = -math.rad(60),
                out_max = math.rad(60),
                deadband = math.rad(0.05),
            },

            rate = {
                kp = 5.0,
                ki = 0.0,
                kd = 0.0,
                i_min = -2.0,
                i_max = 2.0,
                out_min = -8.0,
                out_max = 8.0,
                deadband = 0.01,
            },
        },
    },
}

return config
