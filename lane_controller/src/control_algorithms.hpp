#ifndef SRC_LANE_CONTROLLER_INCLUDE_LANE_CONTROLLER_CONTROL_ALGORITHMS_HPP_
#define SRC_LANE_CONTROLLER_INCLUDE_LANE_CONTROLLER_CONTROL_ALGORITHMS_HPP_


typedef struct {

	double kp;
	double ki;
	double kd;

	double prev_err;
	double prev_intg;

} T_PID;

typedef struct {

	double k;

	int prev;

} T_PT1;

extern double pid_control(double err, T_PID *control_struct);
extern int filter_pt1(int u, T_PT1 *control_struct);

extern void reset_pid_control(T_PID *control_struct);
extern void reset_pt1(T_PT1 *control_struct);

#endif /* SRC_LANE_CONTROLLER_INCLUDE_LANE_CONTROLLER_CONTROL_ALGORITHMS_HPP_ */
