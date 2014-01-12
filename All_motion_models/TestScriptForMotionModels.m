% This script runs some test on the current motion model
a = state.sample_a_valid_state();
a = a.draw();
b = state.sample_a_valid_state();
b = b.draw();

traj = MotionModel_class.generate_VALID_open_loop_point2point_traj(a,b);

MotionModel_class.draw_nominal_traj(traj)