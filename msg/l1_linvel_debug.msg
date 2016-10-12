#uint64 timestamp
uint16 seq_id
float32[3] vel  # linear velocity estimate
float32[4] rpm  # RPM estimate
float32[3] dst  # acceleration disturbance estimate
float32[3] lpd # low pass filtered acceleration disturbance estimate
