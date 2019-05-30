# Estimated external disturbance(force or torque)

#uint64 timestamp_estimate	    # the timestamp when the estimate is received

# Estimated disturbance in body frame
float32[3] esti_dist

# Intermediate variables
float32[3] ft_hat
float32[3] ft_motor
float32[3] ft_motor_filter

# TOPICS estimate_disturb estimate_force estimate_torque

