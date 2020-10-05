% load the initial condition
omega_0 = [0;0;0];
euler0 = [0.0; 5.0/57.3; 30.0/57.3];
quaternion0  = angle2quat(euler0(3), euler0(2), euler0(1));
