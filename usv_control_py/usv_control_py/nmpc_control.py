#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64
import casadi as ca
import numpy as np

class NMPCPathFollower(Node):
    def __init__(self):
        super().__init__('nmpc_path_follower')
        
        # Declare and get parameters
        # Declare and get parameters
        acceptance_radius_descriptor = ParameterDescriptor(description='Radius to consider a waypoint reached')
        target_speed_descriptor = ParameterDescriptor(description='Target speed for the NMPC controller')
        wind_speed_descriptor = ParameterDescriptor(description='Wind speed')
        wind_deg_descriptor = ParameterDescriptor(description='Wind direction in degrees')

        self.declare_parameter('acceptance_radius', 7.0, acceptance_radius_descriptor)
        self.declare_parameter('target_speed', 1.5, target_speed_descriptor)
        self.declare_parameter('wind_speed', 0.0, wind_speed_descriptor)
        self.declare_parameter('wind_deg', 240, wind_deg_descriptor)

        # Model parameters declarations
        self.declare_parameter('m11', 180.0)
        self.declare_parameter('m22', 180.0)
        self.declare_parameter('m33', 466.0)
        self.declare_parameter('Xu', -100.0)
        self.declare_parameter('Xuu', -150.0)
        self.declare_parameter('Yv', -100.0)
        self.declare_parameter('Yvv', -100.0)
        self.declare_parameter('Nr', -800.0)
        self.declare_parameter('Nrr', -800.0)
        self.declare_parameter('B_width', 2.06)
        self.declare_parameter('N', 20)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('cx', 0.5)
        self.declare_parameter('cy', 0.5)
        self.declare_parameter('cn', 0.33)
        self.declare_parameter('A_Fw', 1.5)
        self.declare_parameter('A_Lw', 7.0)
        self.declare_parameter('L_boat', 4.9)

        self.acceptance_radius = self.get_parameter('acceptance_radius').get_parameter_value().double_value
        self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value
        self.wind_speed = self.get_parameter('wind_speed').get_parameter_value().double_value
        self.wind_deg = self.get_parameter('wind_deg').get_parameter_value().integer_value

        # Model parameters retrieval
        self.m11 = self.get_parameter('m11').get_parameter_value().double_value
        self.m22 = self.get_parameter('m22').get_parameter_value().double_value
        self.m33 = self.get_parameter('m33').get_parameter_value().double_value
        self.Xu = self.get_parameter('Xu').get_parameter_value().double_value
        self.Xuu = self.get_parameter('Xuu').get_parameter_value().double_value
        self.Yv = self.get_parameter('Yv').get_parameter_value().double_value
        self.Yvv = self.get_parameter('Yvv').get_parameter_value().double_value
        self.Nr = self.get_parameter('Nr').get_parameter_value().double_value
        self.Nrr = self.get_parameter('Nrr').get_parameter_value().double_value
        self.B_width = self.get_parameter('B_width').get_parameter_value().double_value
        self.N = self.get_parameter('N').get_parameter_value().integer_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value
        self.cn = self.get_parameter('cn').get_parameter_value().double_value
        self.A_Fw = self.get_parameter('A_Fw').get_parameter_value().double_value
        self.A_Lw = self.get_parameter('A_Lw').get_parameter_value().double_value
        self.L_boat = self.get_parameter('L_boat').get_parameter_value().double_value

        self.current_wp_idx = 0
        self.state = np.zeros(6) # [x, y, psi, u, v, r]
        self.pa = 1.225
        self.waypoints = []
        # Wind
        self.Vw = self.wind_speed
        self.beta_w = math.radians(self.wind_deg)

        
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Path,'planned_path',self.pathCallback, 10)
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        self.setup_nmpc()
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("NMPC WAM-V initiated")

    def setup_nmpc(self):
        self.opti = ca.Opti()
        self.X = self.opti.variable(6, self.N + 1)
        self.U = self.opti.variable(2, self.N)
        self.X0 = self.opti.parameter(6)
        self.PA = self.opti.parameter(2)
        self.PB = self.opti.parameter(2)
        self.V_REF = self.opti.parameter()

        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            
            # wind + motors
            tau_m = ca.vertcat(con[0] + con[1], 0, (con[1] - con[0]) * (self.B_width / 2))
            
            Vrw_x = st[3] - self.Vw * ca.cos(self.beta_w - st[2])
            Vrw_y = st[4] - self.Vw * ca.sin(self.beta_w - st[2])
            Vrw = ca.sqrt(Vrw_x**2 + Vrw_y**2 + 1e-6)
            q = 0.5 * self.pa * Vrw**2
            
            tau_w = ca.vertcat(
                q * -self.cx * (Vrw_x/Vrw) * self.A_Fw,
                q * -self.cy * (-Vrw_y/Vrw) * self.A_Lw,
                q * self.cn * (2 * (Vrw_x/Vrw) * (-Vrw_y/Vrw)) * self.A_Lw * self.L_boat
            )
            tt = tau_m + tau_w

            # Acelerations
            du = (tt[0] + (self.Xu + self.Xuu*ca.fabs(st[3]))*st[3] + self.m22*st[5]*st[4]) / self.m11
            dv = (tt[1] + (self.Yv + self.Yvv*ca.fabs(st[4]))*st[4] - self.m11*st[3]*st[5]) / self.m22
            dr = (tt[2] + (self.Nr + self.Nrr*ca.fabs(st[5]))*st[5]) / self.m33

            dx = ca.vertcat(st[3]*ca.cos(st[2]) - st[4]*ca.sin(st[2]), 
                            st[3]*ca.sin(st[2]) + st[4]*ca.cos(st[2]), 
                            st[5], du, dv, dr)
            self.opti.subject_to(self.X[:, k+1] == st + self.dt * dx)

        # Cost
        obj = 0
        line_vec = self.PB - self.PA
        line_unit = line_vec / (ca.norm_2(line_vec) + 1e-6)
        line_angle = ca.atan2(line_vec[1], line_vec[0])

        for k in range(self.N):
            v_rel = self.X[0:2, k] - self.PA
            cte = v_rel[0]*line_unit[1] - v_rel[1]*line_unit[0]
            yaw_err = ca.atan2(ca.sin(self.X[2, k]-line_angle), ca.cos(self.X[2, k]-line_angle))
            
            obj += 3000 * cte**2 + 2000 * yaw_err**2 + 5000 * (self.X[3, k] - self.V_REF)**2
            obj += 0.01 * (self.U[0, k]**2 + self.U[1, k]**2)
            # obj += 5000 * cte**2 + 1000 * yaw_err**2 + 1000 * (self.X[3, k] - self.V_REF)**2
            # obj += 0.01 * (self.U[0, k]**2 + self.U[1, k]**2)
            
        self.opti.minimize(obj)
        self.opti.subject_to(self.opti.bounded(-2000, self.U, 2000))
        self.opti.subject_to(self.X[:, 0] == self.X0)
        self.opti.solver('ipopt', {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": 30})

    def odom_callback(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        self.state[0], self.state[1] = p.x, p.y
        self.state[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.state[3], self.state[4], self.state[5] = v.linear.x, v.linear.y, v.angular.z

    def pathCallback(self, msg):
        self.waypoints = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])   

    def control_loop(self):
        if len(self.waypoints) < 2:
            return
        
        pa = self.waypoints[self.current_wp_idx]
        pb = self.waypoints[(self.current_wp_idx + 1) % len(self.waypoints)]

    
        
        # Calcs to log
        line_vec = pb - pa
        line_dist = np.linalg.norm(line_vec) + 1e-6
        line_unit = line_vec / line_dist
        line_angle = math.atan2(line_vec[1], line_vec[0])
        
        # (CTE)
        v_rel = self.state[0:2] - pa
        cte = v_rel[0] * line_unit[1] - v_rel[1] * line_unit[0]
        
        # (Yaw Error)
        yaw_err = self.state[2] - line_angle
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
        
        if np.linalg.norm(self.state[0:2] - pb) < self.acceptance_radius:
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)

        self.opti.set_value(self.X0, self.state)
        self.opti.set_value(self.PA, pa); self.opti.set_value(self.PB, pb)
        self.opti.set_value(self.V_REF, self.target_speed)
        
        try:
            sol = self.opti.solve()
            u = sol.value(self.U[:, 0])
        except:
            u = self.opti.debug.value(self.U[:, 0])
            
        self.pub_left.publish(Float64(data=float(u[0])))
        self.pub_right.publish(Float64(data=float(u[1])))

        self.get_logger().info(
            f"WP:{self.current_wp_idx} | "
            f"CTE: {cte:6.2f}m | "
            f"YAW_ERR: {math.degrees(yaw_err):6.1f}° | "
            f"SPD: {self.state[3]:.2f}m/s | "
            f"L:{u[0]:.0f} R:{u[1]:.0f}"
        )

def main():
    rclpy.init()
    rclpy.spin(NMPCPathFollower())
    rclpy.shutdown()

if __name__ == '__main__':
    main()