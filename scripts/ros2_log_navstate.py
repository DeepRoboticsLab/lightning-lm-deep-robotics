#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from datetime import datetime
from lightning.msg import NavState  # 根据你的 msg/NavState.msg 确认包名为 lightning

# source install/local_setup.bash && python3 src/lightning-lm/scripts/ros2_log_navstate.py
class NavStateLogger(Node):
    
    def __init__(self):
        super().__init__('nav_state_logger')
        
        # 订阅话题
        self.subscription = self.create_subscription(
            NavState,
            '/lightning/nav_state',
            self.listener_callback,
            10)
        
        # 创建 log 目录
        log_dir = "data/libraryf_0"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
        # 按照当前时间命名输出文件
        file_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(log_dir, f"nav_state_{file_timestamp}.txt")
        
        # 写入表头，增加单位注释（4位精度）
        self.f = open(self.filename, 'w')
        header = "timestamp,px,py,pz,qx,qy,qz,qw,vx,vy,vz,confidence,pose_is_ok\n"
        self.f.write(header)
        
        self.get_logger().info(f'🚀 开始记录 /lightning/nav_state 到 {self.filename}')

    def listener_callback(self, msg):
        # 提取时间戳
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 提取 Pose
        p = msg.pose.position
        q = msg.pose.orientation
        
        # 提取 Velocity
        v = msg.velocity
        
        # 格式化输出字符串：.4f 表示 4 位小数精度
        line = (f"{ts:.6f},"
                f"{p.x:.4f},{p.y:.4f},{p.z:.4f},"
                f"{q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f},"
                f"{v.x:.4f},{v.y:.4f},{v.z:.4f},"
                f"{msg.confidence:.4f},{int(msg.pose_is_ok)}\n")
        
        self.f.write(line)
        self.f.flush()  # 实时写入磁盘

    def __del__(self):
        if hasattr(self, 'f'):
            self.f.close()

def main(args=None):
    rclpy.init(args=args)
    node = NavStateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 停止记录。')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
