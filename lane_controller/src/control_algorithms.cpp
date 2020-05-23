#include "control_algorithms.hpp"


/* ---------------------------
 * ---------------------------
 */
double pid_control(double err, T_PID *control_struct)
{
	double y, intg, diff = 0.0f;

	intg = control_struct->prev_err + control_struct->prev_intg;
	diff = err - control_struct->prev_err;

	y = control_struct->kp*err + control_struct->ki*intg + control_struct->kd*diff;

  /* saturate integrator */
  if(intg > 1) intg = 1;
  if(intg < -1) intg = -1;

  /* saturate output */
  if(y > 1) y = 1;
  if(y < -1) y = -1;

	control_struct->prev_err = err;
	control_struct->prev_intg = intg;

	return (y);

} /* pid_control */


/* -------------------------------------------------------------------------
 * ------------------------------------------------------------------------- */
int filter_pt1(int u, T_PT1 *control_struct)
{
	double tmp_u = static_cast<double>(u);
	double tmp_prev = static_cast<double>(control_struct->prev);

	tmp_prev = tmp_prev + control_struct->k * (tmp_u - tmp_prev);
	control_struct->prev = tmp_prev;

	return (static_cast<int>(tmp_prev));

} /* filter_pt1 */


/* -------------------------------------------------------------------------
 * ------------------------------------------------------------------------- */
void reset_pid_control(T_PID *control_struct)
{
	control_struct->prev_err = 0.0f;
	control_struct->prev_intg = 0.0f;

} /* reset_pid_control */


/* -------------------------------------------------------------------------
 * ------------------------------------------------------------------------- */
void reset_pt1(T_PT1 *control_struct)
{
	control_struct->prev = 0.0f;

} /* reset_pt1 */

