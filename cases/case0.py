params = {
  "meteorites": [
    {
      "c_pos_x": -0.098000476639321444,
      "c_vel_x": 0.00059304512389884,
      "c_accel": 0.,
      "c_pos_y": 0.999999999,
      "c_vel_y": -0.012,
      "t_start": 0,
      "id": 1000
    }
  ],
  "initial_laser_state": {
    "h": 1.5707963267948966,
    "hp": 6,
    "laser_shots_remaining": 60
  },
  "accel_corr_factor_s": 0.3333333333333333,
  "prob_hit_destroys": 0.75,
  "num_laser_shots": 60,
  "laser_effectiveness_distance": 1.1,
  "in_bounds": {
    "x_bounds": [
      -1.0,
      1.0
    ],
    "y_bounds": [
      -1.0,
      1.0
    ]
  },
  "min_dist": 0.01,
  "noise_sigma_x": 0.0,
  "noise_sigma_y": 0.0,
  "nsteps": 500,
  "dt": 0.1,
  "_args": {
    "outfile": "../provided_to_students/cases/case0.py",
    "turret_x": 0.0,
    "turret_hp": 6,
    "num_laser_shots": 60,
    "t_past": 0,
    "t_future": 1,
    "t_step": 6,
    "noise_sigma_x": 0.0,
    "noise_sigma_y": 0.0,
    "min_y_init": 0.31,
    "max_y_init": 2.5,
    "nsteps": 500,
    "dt": 0.1,
    "meteorite_c_pos_max": 0.5,
    "meteorite_c_vel_max": 0.1,
    "meteorite_c_accel_max": 0.005,
    "min_dist": 0.01,
    "max_angle_change": 0.0873,
    "seed": 133
  }
}
