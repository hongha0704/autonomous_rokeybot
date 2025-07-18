from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
import time

'''추가'''
import threading
import matplotlib.pyplot as plt
from collections import deque
from datetime import datetime
import os


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        # PD control related variables
        self.last_error = 0 
        # self.MAX_VEL = 0.1  ## 기존 코드
        self.MAX_VEL = 0.20    ## 수정 코드

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()

        # 감속 지속 시간 설정
        self.slowdown_start_time = None
        self.slowdown_duration = 10

        # 횡단보도 일시정지 모드, 일시 정지 여부
        self.crosswork_stop_mode = False
        self.crosswork_stop_bool = False

        '''추가'''
        self.error_list = deque(maxlen=1000)  # 최근 error 저장
        self.time_list = deque(maxlen=1000)
        self.start_time = time.time()
        # 실시간 그래프 쓰레드 시작
        self.graph_thread = threading.Thread(target=self.plot_error_realtime)
        self.graph_thread.daemon = True
        self.graph_thread.start()


    '''추가 - 중심선 에러를 실시간으로 시각화하는 함수'''
    def plot_error_realtime(self):
        plt.ion()
        fig, ax = plt.subplots()
        line, = ax.plot([], [], 'r-', label='Lane Error')
        zero_line = ax.axhline(0, color='blue', linestyle='--', linewidth=1, label='y = 0')

        ax.set_ylim(-30, 30)  # error 값의 예상 범위
        ax.set_xlim(0, 100)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error')
        ax.set_title('Real-time Lane Error Plot')
        ax.legend()

        # 초 단위 저장 주기
        save_interval = 1.0
        last_save_time = time.time()
        
        # 저장 경로
        save_dir = '/home/hongha/turtlebot3_ws/src/turtlebot3_autorace/turtlebot3_autorace_mission/turtlebot3_autorace_mission'

        while True:
            if len(self.time_list) > 0:
                line.set_xdata(self.time_list)
                line.set_ydata(self.error_list)

                # x축 범위 자동 조정
                ax.set_xlim(max(3, self.time_list[0]), self.time_list[-1] + 0.1)
                
                # y축 자동 조정도 가능하지만 고정이 더 좋을 수 있음
                ax.relim()
                ax.autoscale_view()

                fig.canvas.draw()
                fig.canvas.flush_events()

                # 주기적으로 이미지 저장
                current_time = time.time()
                if current_time - last_save_time > save_interval:
                    # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    # filename = f'lane_error_plot_{timestamp}.png'
                    filename = f'lane_error_plot.png'
                    full_path = os.path.join(save_dir, filename)
                    fig.savefig(full_path)
                    print(f"[✅ 저장됨] {full_path}")
                    last_save_time = current_time

                time.sleep(0.1)


    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data


    '''메인 함수. 속도를 조절하고 퍼블리시 하는 함수'''
    def callback_follow_lane(self, desired_center):
        """
        Receive lane center data to generate lane following control commands.
        If avoidance mode is enabled, lane following control is ignored.
        """
        if self.avoid_active:
            return

        center = desired_center.data
        error = center - 500
        # error = center - 320  ##### 중앙선 320으로 바꿔야함

        # Kp = 0.0025
        # Kd = 0.007

        Kp = 0.024
        Kd = 0.0005

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        '''추가'''
        # 그래프에 사용할 데이터 추가
        self.error_list.append(error)
        self.time_list.append(time.time() - self.start_time)

        # 속도 계산
        twist = Twist()
        # twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)  ## 기존 코드(상한값 0.05)
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.20)    ## 수정 코드
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)

        # /control/cmd_vel 퍼블리시
        self.pub_cmd_vel.publish(twist)
        self.get_logger().info(f'linear.x = {twist.linear.x:.3f}, angular.z = {twist.angular.z:.3f}')  # 속도 로그 출력
        

    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)


    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')


    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
