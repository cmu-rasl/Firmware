# TOPICS att_ctrl_debug
float32[6] rpm   # motor rpm out values assuming quadrotor
float32[3] eOm   # angular velocity error
float32[3] Omd   # desired angular velocity
float32[3] eR    # orientation error
float32[9] Rd    # desired orientation (populated in row-major order)
float32[9] R    # actual orientation (populated in row-major order)
