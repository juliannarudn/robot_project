import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from scipy.linalg import solve_continuous_are

class LQGController(Node):
    def __init__(self):
        super().__init__('lqg_controller')
        
        # Параметры цели
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', -1.0)
        self.declare_parameter('goal_theta', 0.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.8)
        
        self.goal = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_theta').value
        ])
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        
        # Состояние робота: [x, y, theta]
        self.state = np.zeros(3)
        
        # Оценка состояния (фильтр Калмана)
        self.estimated_state = np.zeros(3)
        self.P = np.eye(3)  # ковариация ошибки оценки
        
        # Матрицы шумов (подберите экспериментально)
        self.Q_kalman = np.diag([0.1, 0.1, 0.05])   # шум процесса
        self.R_kalman = np.diag([0.05, 0.05, 0.03])  # шум измерений
        
        # Матрицы весов LQR
        self.Q_lqr = np.diag([1.0, 1.0, 0.5])        # штраф на ошибку
        self.R_lqr = np.diag([0.1, 0.05])            # штраф на управление
        
        # Подписка на одометрию
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Публикация управления
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Таймер управления (20 Гц)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('LQG controller started')

    def odom_callback(self, msg):
        # Измеренное состояние из одометрии
        z = np.zeros(3)
        z[0] = msg.pose.pose.position.x
        z[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        z[2] = math.atan2(siny_cosp, cosy_cosp)
        
        # Шаг фильтра Калмана
        # 1) Прогноз: (динамика отсутствует, используем простое предсказание)
        # Предполагаем, что движение медленное, состояние почти не меняется между измерениями
        self.estimated_state = self.estimated_state  # здесь можно добавить модель, если нужно
        self.P = self.P + self.Q_kalman
        
        # 2) Обновление
        K = self.P @ np.linalg.inv(self.P + self.R_kalman)
        self.estimated_state = self.estimated_state + K @ (z - self.estimated_state)
        self.P = (np.eye(3) - K) @ self.P

    def control_loop(self):
        # Ошибка между оценкой состояния и целью
        error = self.goal - self.estimated_state
        error[2] = math.atan2(math.sin(error[2]), math.cos(error[2]))
        
        # Если цель достигнута, останавливаем
        if np.linalg.norm(error[:2]) < 0.1 and abs(error[2]) < 0.1:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
        
        # Локальная линеаризация в текущей оценке
        theta = self.estimated_state[2]
        v_current = 0.0  # можно взять из одометрии, но для простоты 0
        A = np.array([
            [0, 0, -v_current * math.sin(theta)],
            [0, 0,  v_current * math.cos(theta)],
            [0, 0, 0]
        ])
        B = np.array([
            [math.cos(theta), 0],
            [math.sin(theta), 0],
            [0, 1]
        ])
        
        # Решаем LQR для текущей линеаризации
        try:
            K_lqr = self.lqr(A, B, self.Q_lqr, self.R_lqr)
            u = -K_lqr @ error
        except:
            # fallback на ПД
            u = self.pd_control(error)
        
        linear = np.clip(u[0], -self.max_linear, self.max_linear)
        angular = np.clip(u[1], -self.max_angular, self.max_angular)
        
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
    
    def lqr(self, A, B, Q, R):
        """Вычисляет матрицу усилений LQR для непрерывной системы."""
        S = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ (B.T @ S)
        return K
    
    def pd_control(self, error):
        """Упрощённый ПД-регулятор (fallback)"""
        kp_lin = 0.5
        kp_ang = 0.8
        linear = kp_lin * error[0]
        angular = kp_ang * error[2]
        return np.array([linear, angular])

def main(args=None):
    rclpy.init(args=args)
    node = LQGController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
