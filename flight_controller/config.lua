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
}

config.navigation = {
    arrival_radius = 5.0,
    waypoint_radius = 8.0,
    climb_tolerance = 1.0,
    altitude_tolerance = 1.0,
    heading_tolerance = math.rad(5),
    horizontal_speed_tolerance = 0.5,
    vertical_speed_tolerance = 0.3,
    heading_rate_tolerance = math.rad(3),
    approach_distance = 40.0,

    waypoints = {
        {
            id = "home",
            name = "Home",
            position = { x = -213, y = 81, z = 264 },

            approaches = {
                {
                    id = "home_west",
                    name = "Home West Approach",
                    heading = math.rad(-90),
                    distance = 40.0,
                    altitude = 100,
                    finalAltitude = 81,
                },
            },

            hold = {
                altitude = 81,
                heading = math.rad(-90),
            },
        },
    },
}

config.hardware = {
    rotor = {
        modem_side = "front",
        upper_bearing = "swivel_bearing_4",
        lower_bearing = "swivel_bearing_5",
        blade_mount = {
            [1] = 0.0,
            [2] = -math.pi / 2,
            [3] = math.pi,
            [4] = math.pi / 2,
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
        phase_offset_upper = math.pi,
        phase_offset_lower = 0,
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
        tilt_compensation = {
            min_factor = 0.5,
        },
    },

    vertical = {
        target_rate = 5.0,
        feedforward = {
            gain = 0.5,
            bias = 1.0,
        },
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

        rate_feedforward = {
            roll = {
                gain = 6.2,
                bias = -0.03,
            },
            pitch = {
                gain = 6.8,
                bias = -0.33,
            },
            yaw = {
                gain = 1.5,
            },
        },
    },

    attitude_allocator = {
        enabled = true,
        model = "affine_tensor",

        base_matrix = {
            { 1.003055,  0.050525, -0.006006 },
            { -0.039758, 1.004566, 0.015211 },
            { -0.330290, 0.005378, 1.000567 },
        },

        terms = {
            { out = "roll", attitude = "pitch", input = "yaw", gain = -0.615553 },
        },

        attitude_limit_deg = {
            roll = 30.0,
            pitch = 25.0,
        },
    },

    output_limits = {
        roll_min = -8.0,
        roll_max = 8.0,
        pitch_min = -8.0,
        pitch_max = 8.0,
        yaw_min = -8.0,
        yaw_max = 8.0,
    },

    heading = {
        lookahead_rate = math.rad(60),
        lookahead_time_constant = 0.70,
        yaw_priority = 0.45,
        lock = {
            rate_deadband = math.rad(2),
            relock_timeout = 0.6,
        },
    },

    position_hold = {
        velocity_feedforward = {
            forward = {
                gain_neg = 0.0200,
                gain_pos = 0.0155,
            },
            right = {
                gain_neg = 0.0165,
                gain_pos = 0.0175,
            },
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
                out_min = 0.0,
                out_max = 10.0,
                deadband = 0.03,
            },
        },

        position = {
            forward = {
                kp = 0.40,
                ki = 0.12,
                kd = 0.20,
                i_min = -0.42,
                i_max = 0.42,
                out_min = -20.0,
                out_max = 20.0,
                deadband = 0.05,
            },

            right = {
                kp = 0.45,
                ki = 0.12,
                kd = 0.25,
                i_min = -0.75,
                i_max = 0.75,
                out_min = -20.0,
                out_max = 20.0,
                deadband = 0.05,
            },
        },

        velocity = {
            forward = {
                kp = 0.050,
                ki = 0.020,
                kd = 0.00,
                i_min = -0.4,
                i_max = 0.4,
                out_min = -math.rad(25),
                out_max = math.rad(25),
                deadband = 0.05,
            },

            right = {
                kp = 0.070,
                ki = 0.025,
                kd = 0.00,
                i_min = -0.4,
                i_max = 0.4,
                out_min = -math.rad(30),
                out_max = math.rad(30),
                deadband = 0.05,
            },
        },

        attitude = {
            roll = {
                angle = {
                    kp = 1.80,
                    ki = 0.18,
                    kd = 0.05,
                    i_min = -math.rad(1.0),
                    i_max = math.rad(1.0),
                    out_min = -math.rad(90),
                    out_max = math.rad(90),
                    deadband = math.rad(0.05),
                },

                rate = {
                    kp = 1.0,
                    ki = 1.5,
                    kd = 0.0,
                    i_min = -0.15,
                    i_max = 0.15,
                    out_min = -8.0,
                    out_max = 8.0,
                    deadband = math.rad(0.05),
                },
            },

            pitch = {
                angle = {
                    kp = 1.25,
                    ki = 0.20,
                    kd = 0.05,
                    i_min = -math.rad(1.0),
                    i_max = math.rad(1.0),
                    out_min = -math.rad(90),
                    out_max = math.rad(90),
                    deadband = math.rad(0.05),
                },

                rate = {
                    kp = 3.5,
                    ki = 1.4,
                    kd = 0.0,
                    i_min = -0.15,
                    i_max = 0.15,
                    out_min = -12.0,
                    out_max = 12.0,
                    deadband = math.rad(0.05),
                },
            },

            yaw = {
                angle = {
                    kp = 0.85,
                    ki = 0.0,
                    kd = 0.25,
                    i_min = -0.5,
                    i_max = 0.5,
                    out_min = -math.rad(60),
                    out_max = math.rad(60),
                    deadband = math.rad(0.05),
                },

                rate = {
                    kp = 6.5,
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
    },
}

return config
