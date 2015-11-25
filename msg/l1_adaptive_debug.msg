uint64 timestamp
uint16 seq_id
float32[3] avl_hat  # angular velocity estimate
float32[3] dst_hat  # disturbance estimate
float32[3] ang_vel  # angular velocity obtained from R
float32[3] lpd      # disturbance rate correction
float32[3] rates    # body rates obtained from gyro
