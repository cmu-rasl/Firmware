uint64 timestamp
uint16 seq_id
float32[4] rpm                # motor rpm values (assumes quadrotor model)
float32[3] ang_vel_err        # angular velocity error
float32[3] ang_vel_des        # desired angular velocity
float32[3] orientation_err    # orientation error
float32[9] orientation_des    # desired orientation (populated in row-major order)
float32[9] orientation_act    # actual orientation (populated in row-major order)
float32    body_cmd_fbz       # thrust component of cascaded command
float32[4] mb                 # torque
