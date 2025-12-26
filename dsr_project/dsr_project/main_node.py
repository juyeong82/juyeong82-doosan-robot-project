#!/usr/bin/env python3
# board_main_coordinator.py
# 역할: GUI의 명령을 받아 로봇 동작 순서(Pick -> Erase -> ...)를 제어

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from my_robot_interfaces.action import BrushingAction

class BoardMainCoordinator(Node):
    def __init__(self):
        super().__init__('board_main_coordinator')
        
        # --- Action Clients 설정 ---
        self.clients = {
            "DOMA_PICK": ActionClient(self, BrushingAction, '/dsr01/do_doma_pick_action'),
            "ERASING":   ActionClient(self, BrushingAction, '/dsr01/do_eraser_action'),
            "BRUSHING":  ActionClient(self, BrushingAction, '/dsr01/do_brushing_action'),
            "OILING":    ActionClient(self, BrushingAction, '/dsr01/do_oiling_action'),
            "DOMA_PLACE":ActionClient(self, BrushingAction, '/dsr01/do_doma_place_action')
        }

        # GUI 명령 수신
        self.create_subscription(String, '/main_task_cmd', self.cmd_callback, 10)
        
        self.busy = False
        self.pending_cmd = None
        self.get_logger().info("BoardMainCoordinator 준비 완료 (v3.0)")

    def cmd_callback(self, msg):
        cmd = msg.data
        if self.busy:
            self.get_logger().warn(f"작업 중입니다. 명령 무시: {cmd}")
        else:
            self.get_logger().info(f"명령 수신: {cmd}")
            self.pending_cmd = cmd

    def run_action(self, name):
        """단일 액션 실행 (Blocking)"""
        client = self.clients.get(name)
        if not client: return False
        
        self.get_logger().info(f"[{name}] 서버 대기중...")
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"[{name}] 서버 응답 없음")
            return False
            
        goal = BrushingAction.Goal()
        goal.start_task = True
        
        self.get_logger().info(f"[{name}] 시작 요청 ->")
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f"[{name}] 거부됨")
            return False
            
        # 결과 대기
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        result = res_future.result().result
        if result.complete_task:
            self.get_logger().info(f"[{name}] ✅ 완료 ({result.total_duration:.1f}s)")
            return True
        else:
            self.get_logger().error(f"[{name}] ❌ 실패")
            return False

    def run_sequence(self, tasks):
        self.busy = True
        try:
            for task_name in tasks:
                if not self.run_action(task_name):
                    self.get_logger().error("시퀀스 중단됨")
                    break
        finally:
            self.busy = False

    def run_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if not self.busy and self.pending_cmd:
                cmd = self.pending_cmd
                self.pending_cmd = None
                
                # 명령에 따른 시퀀스 정의
                if cmd == "START_ALL":
                    self.run_sequence(["DOMA_PICK", "ERASING", "BRUSHING", "OILING", "DOMA_PLACE"])
                elif cmd == "START_DOMA_PICK": self.run_sequence(["DOMA_PICK"])
                elif cmd == "START_ERASING":   self.run_sequence(["ERASING"])
                elif cmd == "START_BRUSHING":  self.run_sequence(["BRUSHING"])
                elif cmd == "START_OILING":    self.run_sequence(["OILING"])
                elif cmd == "START_DOMA_PLACE":self.run_sequence(["DOMA_PLACE"])

def main():
    rclpy.init()
    node = BoardMainCoordinator()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
