import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import casadi as ca
import numpy as np
import math

# Global variable to store the drone's position
current_position = None
publish_rate = 10
# Callback to get the drone's current position
def pose_callback(data):
    global current_position
    current_position = (data.pose.pose.position.x,
                        data.pose.pose.position.y,
                        data.pose.pose.position.z)

# MPC control function
def mpc_control(t, T, x0_, N_):
    opti = ca.Opti()

    # Control input: vertical velocity vz
    opt_controls = opti.variable(N_, 1)
    vz = opt_controls

    # State variable: position z
    opt_states = opti.variable(N_+1, 1)
    z = opt_states

    # Create model
    f = lambda x_, u_: u_[0]

    # Parameters: reference trajectories of the state and input
    opt_u_ref = opti.parameter(N_, 1)
    opt_x_ref = opti.parameter(N_+1, 1)

    # Initial condition
    opti.subject_to(opt_states[0, :] == opt_x_ref[0, :])
    for i in range(N_):
        z_next = opt_states[i, :] + f(opt_states[i, :], opt_controls[i, :]).T * T
        opti.subject_to(opt_states[i+1, :] == z_next)

    # Weight matrix
    Q = np.diag([30.0])
    R = np.diag([1.0])

    # Cost function
    obj = 0
    for i in range(N_):
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

    # Solve the optimization problem
    opti.set_value(opt_x_ref, np.vstack((x0_, np.tile(np.array([[-10.0]]), (N_, 1)))))
    opti.set_value(opt_u_ref, np.tile(np.array([[0.0]]), (N_, 1)))
    sol = opti.solve()

    # Obtain the control input
    u_res = sol.value(opt_controls)
    x_m = sol.value(opt_states)

    return x_m, u_res

if __name__ == "__main__":
    rospy.init_node('mpc_controller')
    rospy.Subscriber("/ground_truth/state", Odometry, pose_callback)  
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Define the desired publish rate (Hz)
    publish_rate = 10  # Change this to the desired publish rate

    # Calculate T to match the publish rate
    T = 1.0 / publish_rate

    rate = rospy.Rate(publish_rate)

    # MPC parameters
    N = 30    # Prediction horizon length

    # Simulation parameters
    t0 = 0
    init_state = np.array([-0.0])
    current_state = init_state.copy()

    while not rospy.is_shutdown():
        # Check if we have received the drone's current position
        if current_position is not None:
            # Call the MPC controller to get the desired trajectory and control inputs
            x_mpc, u_mpc = mpc_control(t0, T, current_state, N)

            # Get the desired vertical velocity command from MPC
            if u_mpc.size > 0:  # Check if the solution is not empty
                vz_ref = u_mpc[0]  # Access the first element of the 1D array

                # Create the Twist message for velocity control
                vel_msg = Twist()
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = vz_ref

                # Publish the control command
                vel_pub.publish(vel_msg)

                # Update time and state
                t0 += T
                current_state = x_mpc[1]  # Update the state to the second element of the MPC result

        rate.sleep()

