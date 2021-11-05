# Frenet PID Module.

import time
import numpy as np
from controller import Controller

class KinFrenetPIDPathFollower(Controller):
	##
	def __init__(self, 
		         N          = 10,            # timesteps in MPC Horizon
		         DT       	= 0.2,           # discretization time between timesteps (s)
		         L_F        = 1.5213,        # distance from CoG to front axle (m)
		         L_R        = 1.4987,        # distance from CoG to rear axle (m)
		         kLA        = 10,            # Lookahead distance (m)
		         k_delta    = 1.,            # Gain for lookahead error
		         kp_acc     = 1.,            # Proportional Gain for velocity error
		         ki_acc     = 0.1,           # Integrator Gain for velocity error
		         INT_MAX    = 5.,            # Integrator error max value to avoid windup
				 AY_MAX     = 3.,
				 AX_MAX     =  5.0,		
		         AX_MIN     = -10.0,         # min/max longitudinal acceleration constraint (m/s^2) 
		         DF_MAX     =  30*np.pi/180,
		         DF_MIN     = -30*np.pi/180, # min/max front steer angle constraint (rad)
		         AX_DOT_MAX  =  1.5,
		         AX_DOT_MIN  = -1.5,          # min/max longitudinal jerk constraint (m/s^3)
		         DF_DOT_MAX =  30*np.pi/180,
		         DF_DOT_MIN = -30*np.pi/180, # min/max front steer angle rate constraint (rad/s)		         
				 ):

		for key in list(locals()):
			if key == 'self':
				pass
			else:
				setattr(self, '%s' % key, locals()[key])

		self.global_state = np.array([0., 0., 0., 0.])
		self.global_ref   = np.zeros((N+1, 4))

		self.z_curr   = np.array([0., 0., 0., 0.]) # s0, ey0, epsi0, v0
		self.curv_ref = np.zeros(self.N)
		self.v_ref    = 1.
		self.u_prev   = np.array([0., 0.]) 

		self.int_error = 0.

	def _get_frenet_trajectory(self, u_mpc):
		z_mpc_frenet = np.ones((1 + u_mpc.shape[0], 4)) * np.nan
		z_mpc_frenet[0, :] = self.z_curr

		for ind, u in enumerate(u_mpc):
			s, ey, ep, v = z_mpc_frenet[ind, :]
			u_acc, u_df = u

			beta = np.arctan( self.L_R / (self.L_F + self.L_R) * np.tan(u_df) )
			dyawdt = v / self.L_R * np.sin(beta)
			dsdt = v * np.cos(ep+beta) / (1 - ey * self.curv_ref[ind] )
			
			sn  = s  + self.DT * (dsdt)  
			eyn = ey + self.DT * (v * np.sin(ep + beta)) 
			epn = ep + self.DT * (dyawdt - dsdt * self.curv_ref[ind])
			vn  = v  + self.DT * (u_acc)

			z_mpc_frenet[ind+1, :] = [sn, eyn, epn, vn]

		return z_mpc_frenet

	def _get_global_trajectory(self, u_mpc):
		z_mpc = np.ones((1 + u_mpc.shape[0], 4)) * np.nan
		z_mpc[0, :] = self.global_state

		for ind, u in enumerate(u_mpc):
			x, y, p, v = z_mpc[ind, :]
			u_acc, u_df = u

			beta = np.arctan( self.L_R / (self.L_F + self.L_R) * np.tan(u_df) )

			xn = x + self.DT * (v * np.cos(p + beta))
			yn = y + self.DT * (v * np.sin(p + beta))
			pn = p + self.DT * (v / self.L_R * np.sin(beta))
			vn = v + self.DT * (u_acc)

			z_mpc[ind+1, :] = [xn, yn, pn, vn]

		return z_mpc

	def solve(self):
		st = time.time()
		
		_, ey, ep, v = self.z_curr

		# Acceleration Control.
		vel_error = self.v_ref - v
		self.int_error += vel_error * self.DT

		if np.abs(self.int_error) > self.INT_MAX:
			self.int_error = 0.

		ax = (self.kp_acc * vel_error  + self.ki_acc * self.int_error)

		# Steering Control.
		
		df = -self.k_delta * (ey + self.kLA * ep)

		# Actuation Limits
		ax_prev, df_prev = self.u_prev


		ax = np.clip(ax, ax_prev + self.AX_DOT_MIN * self.DT, ax_prev + self.AX_DOT_MAX * self.DT)
		ax = np.clip(ax, self.AX_MIN, self.AX_MAX)

		
		df = np.clip(df, df_prev + self.DF_DOT_MIN * self.DT, df_prev + self.DF_DOT_MAX * self.DT)
		df = np.clip(df, self.DF_MIN, self.DF_MAX)
		

		solve_time = time.time() - st

		u_mpc = np.array([ax, df]).reshape(1, 2)
		
		sol_dict = {}
		sol_dict['u_control']    = u_mpc[0,:]                         # control input to apply based on solution
		sol_dict['optimal']      = True                               # whether the solution is optimal or not (not really relevant for PID but used to signal this should be applied)
		sol_dict['solve_time']   = solve_time                         # how long the solver took in seconds
		sol_dict['u_mpc']        = u_mpc                              # solution inputs (here is just a 1 by 2) 
		sol_dict['z_mpc_frenet'] = self._get_frenet_trajectory(u_mpc) # solution states (N+1 by 4, see self.z_dv above)
		
		sol_dict['v_ref_frenet']    = self.v_ref        # velocity reference (scalar)
		sol_dict['curv_ref_frenet'] = self.curv_ref     # curvature reference (N by 1)

		sol_dict['z_mpc'] = self._get_global_trajectory(u_mpc) # get MPC solution trajectory in global frame (N+1, 4)
		sol_dict['z_ref'] = self.global_ref                    # state reference (N by 4)

		sol_dict['sl_mpc'] = np.zeros((u_mpc.shape))

		return sol_dict

	def update(self, update_dict):		
		self.global_state = np.array( [update_dict[key] for key in ['x0', 'y0', 'psi0', 'v0']] )
		self.global_ref   = np.column_stack(( update_dict['x_ref'], 
			                                  update_dict['y_ref'],
			                                  update_dict['psi_ref'],
			                                  update_dict['v_ref'] ))

		self.z_curr   = np.array([update_dict[key] for key in ['s0', 'e_y0', 'e_psi0', 'v0']])

		# Calculate and apply a velocity reference limit from the upcoming curvatures.
		# This is designed assuming the velocity setpoint is constant and not time-varying.
		v_desired     = update_dict['v_ref'][0]
		curvature_max = np.amax(np.fabs(update_dict['curv_ref']))
		v_limit       = np.sqrt(self.AY_MAX/np.fabs(curvature_max))

		self.curv_ref = update_dict['curv_ref']
		self.v_ref    = np.clip(v_desired, -v_limit, v_limit )
		self.u_prev   = np.array([update_dict[key] for key in ['acc_prev', 'df_prev']]) 

if __name__ == '__main__':
	kmpc = KinFrenetMPCPathFollower()
	sol_dict = kmpc.solve()
