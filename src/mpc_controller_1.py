import casadi as ca
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

# Quadrotor parameters
mq = 1          # Mass of the quadrotor [kg]
g = 9.8 
T = 0.02        # Gravity [m/s^2]

def desired_command_and_trajectory(t, T, x0_, N_):
    vz_ref_ = 0.0  # Desired vertical velocity [m/s]
    u_ = np.ones((N_, 1)) * vz_ref_

    z_ref_ = 10.0  # Desired z position [m]
    x_ = np.ones((N_+1, 1)) * x0_
    x_[:, 0] = z_ref_

    return x_, u_
if __name__ == "__main__":
    
    opti = ca.Opti()
    T = 0.02
    N = 30
    # Control input: vertical velocity vz
    opt_controls = opti.variable(N, 1)
    vz = opt_controls

    # State variable: position z
    opt_states = opti.variable(N+1, 1)
    z = opt_states

    # Create model
    f = lambda x_, u_: u_[0]

    # Parameters: reference trajectories of the state and input
    opt_u_ref = opti.parameter(N, 1)
    opt_x_ref = opti.parameter(N+1, 1)

    # Initial condition
    opti.subject_to(opt_states[0, :] == opt_x_ref[0, :])
    for i in range(N):
        z_next = opt_states[i, :] + f(opt_states[i, :], opt_controls[i, :]).T * T
        opti.subject_to(opt_states[i+1, :] == z_next)

    # Weight matrix
    Q = np.diag([30.0])
    R = np.diag([1.0])

    # Cost function
    obj = 0
    for i in range(N):
        state_error_ = opt_states[i, :] - opt_x_ref[i+1, :]
        control_error_ = opt_controls[i, :] - opt_u_ref[i, :]
        obj = obj + ca.mtimes([state_error_, Q, state_error_.T]) + ca.mtimes([control_error_, R, control_error_.T])
    opti.minimize(obj)

    # Boundary and control conditions
    opti.subject_to(opti.bounded(-math.inf, z, 0))
    opti.subject_to(opti.bounded(-1000, vz, 1000))

    opts_setting = {'ipopt.max_iter': 2000,
                    'ipopt.print_level': 0,
                    'print_time': 0,
                    'ipopt.acceptable_tol': 1e-8,
                    'ipopt.acceptable_obj_change_tol': 1e-6}

    opti.solver('ipopt', opts_setting)

    # Simulation parameters
    t0 = 0
    init_state = np.array([-0.0])
    current_state = init_state.copy()
    u0 = np.zeros((N, 1))
    next_controls = np.zeros((N, 1))
    next_states = np.zeros((N+1, 1))
    x_c = []  # Contains the history of the state
    u_c = [u0[0]]
    t_c = [t0]  # For time
    xx = [current_state]
    xr = [next_states[0]]
    ur = [next_controls[0]]
    sim_time = 10.0

    # Initialize ROS node and publisher for cmd_vel
    rospy.init_node('mpc_quadrotor_control')
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1/T)  # Publish at MPC control rate

    # Start MPC
    mpciter = 0
    start_time = time.time()
    index_t = []
    while not rospy.is_shutdown():
        # Set parameters: update the initial state of z (z0)
        opti.set_value(opt_x_ref, np.vstack((current_state, next_states[1:])))
        opti.set_value(opt_u_ref, next_controls)

        # Provide the initial guess of the optimization targets
        opti.set_initial(opt_controls, u0.reshape(N, 1))
        opti.set_initial(opt_states, next_states)

        # Solve the problem
        t_ = time.time()
        sol = opti.solve()
        index_t.append(time.time() - t_)

        # Obtain the control input (desired vertical velocity)
        u_res = sol.value(opt_controls)
        vz_ref = np.ravel(u_res)[0] 
        # Publish the desired vertical velocity as Twist message
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.z = vz_ref
        cmd_vel_pub.publish(cmd_vel_msg)

        # Wait for MPC control rate
        rate.sleep()

        # Update state and control for the next iteration
        t0, current_state = rospy.Time.now().to_sec(), current_state  # Use the actual time and state from the odometry
        next_states, next_controls = desired_command_and_trajectory(t0, T, current_state, N)
        xr.append(next_states[1])
        ur.append(next_controls[0])
        print('iter: ',mpciter,'state: ',-current_state)
        mpciter = mpciter + 1

        # Update iteration count and check for termination
        if mpciter*T >= sim_time:
            break

    # After the loop
    t_v = np.array(index_t)
    print(t_v.mean())
    print((time.time() - start_time)/(mpciter))

    # Plot Attitude Tracking
    plt.figure()
    plt.plot(t_c, -np.array(xr)[:,0], label='ref z')
    plt.plot(t_c, -np.array(xx)[:,0], label='quad z')
    plt.legend()
    plt.xlabel("time [s]")
    plt.ylabel("value [m]")
    plt.grid(True)
    plt.title("Directly tracking")

    # # Plot Thrust force
    # plt.figure()
    # plt.step(t_c, np.array(u_c), label='a')
    # plt.legend()
    # plt.xlabel("time [s]")
    # plt.ylabel("value [N]")
    # plt.grid(True)
    # plt.title("Thrust force")

    plt.show()

