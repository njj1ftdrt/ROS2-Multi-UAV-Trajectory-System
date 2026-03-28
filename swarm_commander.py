#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SwarmCommander(Node):
    def __init__(self):
        super().__init__('swarm_commander_node')
        self.publisher_ = self.create_publisher(Int32, '/swarm_command', 10)
        
        self.print_menu()

    def print_menu(self):
        print('\n================================')
        print(' 🚀 交互式无人机编队系统 🚀')
        print('================================')
        print(' [0] 归位待命 (Hover)')
        print(' [1] 环绕飞行 (Circle) - Day1')
        print(' [2] V字雁阵 (V-Shape) - Day2')
        print(' [3] 一字长蛇阵 (Line) - Day2')
        print(' [4] AI 轨迹预测模式 (AI Tracking) - Day4准备')
        print(' [q] 退出指挥台')
        print('================================')

    def send_command(self, mode):
        msg = Int32()
        msg.data = mode
        self.publisher_.publish(msg)
        self.get_logger().info(f'>> [指令下发] 全集群切换至模式: {mode} <<')

def main(args=None):
    rclpy.init(args=args)
    commander = SwarmCommander()

    try:
        while rclpy.ok():
            cmd = input("\n请下达阵型变换指令 (0-4): ")
            if cmd.lower() == 'q':
                break
            elif cmd in ['0', '1', '2', '3', '4']:
                commander.send_command(int(cmd))
            else:
                print("无效指令！请重新输入。")
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
