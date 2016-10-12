#uint64 timestamp
uint16 seq_id
float32[3] avl  # angular velocity estimate
float32[4] rpm  # RPM estimate
float32[3] dst  # torque disturbance estimate
float32[3] lpd # low pass filtered torque disturbance estimate
