#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
from geometry_msgs.msg import TwistStamped, PoseStamped
import math # For distance calculations

current_state = None
current_pose = PoseStamped()  # Stores current local position (x, y, z)
current_velocity = TwistStamped() # Stores current local velocity (vx, vy, vz)

def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

def velocity_callback(msg):
    global current_velocity
    current_velocity = msg

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def pd_controller(target, current_pos, current_vel, kP, kD, max_cmd, dead_zone=0.03):
    """
    通用 PD 控制器，用於位置控制並輸出速度指令
    target: 目標位置
    current_pos: 當前位置
    current_vel: 當前速度
    kP, kD: 比例與微分增益
    max_cmd: 最大輸出速度限制
    dead_zone: 死區，在目標附近時減少抖動
    """
    error = target - current_pos
    
    # 在死區內，只使用微分項來抑制速度
    if abs(error) < dead_zone:
        if abs(current_vel) < 0.05: # 速度也很小時才真正停止控制
            cmd = 0.0
        else:
            cmd = -kD * current_vel
        return cmd, error
    
    # 標準 PD 控制
    cmd = kP * error - kD * current_vel
    
    # 限制輸出範圍
    cmd = clamp(cmd, -max_cmd, max_cmd)
    return cmd, error

def xyz_pd_controller(target_pos, current_pose, current_vel, kP, kD, max_cmd, dead_zone=0.03):
    """
    三維 PD 控制器，同時控制 X, Y, Z 軸
    target_pos: 目標位置
    current_pose: 當前位置(PoseStamped)
    current_vel: 當前速度(TwistStamped)
    return: TwistStamped 命令
    """
    cmd = TwistStamped()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = 'base_link'
    
    # X 軸控制
    vx_cmd, error_x = pd_controller(target_pos[0], current_pose.pose.position.x,
                                   current_vel.twist.linear.x, kP[0], kD[0], max_cmd[0], dead_zone)
    # Y 軸控制
    vy_cmd, error_y = pd_controller(target_pos[1], current_pose.pose.position.y,
                                   current_vel.twist.linear.y, kP[1], kD[1], max_cmd[1], dead_zone)
    # Z 軸控制
    vz_cmd, error_z = pd_controller(target_pos[2], current_pose.pose.position.z,
                                   current_vel.twist.linear.z, kP[2], kD[2], max_cmd[2], dead_zone)
    cmd.twist.linear.x = vx_cmd
    cmd.twist.linear.y = vy_cmd
    cmd.twist.linear.z = vz_cmd
    cmd.twist.angular.z = 0.0  # 不控制偏航
    
    return cmd, [error_x, error_y, error_z]

def main():
    global current_state, current_pose, current_velocity
    
    rospy.init_node("drone_goto_ab_pd_controller", anonymous=True)
    
    rospy.Subscriber("/mavros/state", State, state_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velocity_callback)

    # 發布速度指令
    velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    
    # 服務客戶端
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    
    rate = rospy.Rate(20) # 20 Hz 控制迴路頻率
    
    rospy.loginfo("等待 MAVROS 連線...")
    while not rospy.is_shutdown() and current_state is None:
        rate.sleep()
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("MAVROS 未連線")
        rate.sleep()
    rospy.loginfo("MAVROS 已連線。")
    
    # 定義零速度指令作為心跳
    zero_velocity_cmd = TwistStamped()
    zero_velocity_cmd.header.frame_id = 'base_link'
    zero_velocity_cmd.twist.linear.x = 0.0
    zero_velocity_cmd.twist.linear.y = 0.0
    zero_velocity_cmd.twist.linear.z = 0.0
    zero_velocity_cmd.twist.angular.z = 0.0 # yaw 速度為0

    # 為 OFFBOARD 模式做準備
    rospy.loginfo("預發送零速度指令，確保 OFFBOARD 模式穩定性...")
    for _ in range(100): # 大約 5 秒
        zero_velocity_cmd.header.stamp = rospy.Time.now()
        velocity_pub.publish(zero_velocity_cmd)
        rate.sleep()
    rospy.loginfo("預發送完成")

    # 進入 OFFBOARD 模式並解鎖
    offb_set_mode_req = SetModeRequest()
    offb_set_mode_req.custom_mode = 'OFFBOARD'

    arm_cmd_req = CommandBoolRequest()
    arm_cmd_req.value = True

    last_req_time = rospy.Time.now()
    
    # 嘗試切換模式直到成功為止，而非只嘗試一次
    rospy.loginfo("嘗試進入 OFFBOARD 模式並解鎖無人機...")
    while not rospy.is_shutdown() and (not current_state.armed or current_state.mode != "OFFBOARD"):
        # 每隔 5 秒嘗試切換模式
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req_time) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode_req).mode_sent:
                rospy.loginfo("✅ OFFBOARD 模式請求已發送")
            last_req_time = rospy.Time.now()
        
        # 模式是 OFFBOARD 後，每隔 5 秒嘗試解鎖
        if current_state.mode == "OFFBOARD" and not current_state.armed and (rospy.Time.now() - last_req_time) > rospy.Duration(5.0):
            if arming_client.call(arm_cmd_req).success:
                rospy.loginfo("✅ 無人機解鎖請求已發送")
            last_req_time = rospy.Time.now()

        # 持續發布心跳
        zero_velocity_cmd.header.stamp = rospy.Time.now()
        velocity_pub.publish(zero_velocity_cmd) 
        rate.sleep()

    rospy.loginfo("無人機已解鎖並進入 OFFBOARD 模式。")

    # 記錄起飛點位置
    takeoff_point = [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z]
    rospy.loginfo(f"起飛點: ({takeoff_point[0]:.2f}, {takeoff_point[1]:.2f}, {takeoff_point[2]:.2f})")

    # 原地起飛 2 公尺
    target_altitude = takeoff_point[2] + 2.0  # 以相對位置來計算
    target_takeoff = [takeoff_point[0], takeoff_point[1], target_altitude]
    
    # PD控制参数
    kP = [0.4, 0.4, 1.2]  # [kP_x, kP_y, kP_z]
    kD = [0.08, 0.08, 0.3]  # [kD_x, kD_y, kD_z]
    max_cmd = [0.5, 0.5, 0.7]  # [max_vx, max_vy, max_vz]

    rospy.loginfo(f"起飛中，目標高度 {target_altitude:.2f} 公尺...")
    
    # 起飛過程使用統一的三維控制
    while not rospy.is_shutdown():
        cmd, errors = xyz_pd_controller(target_takeoff, current_pose, current_velocity, kP, kD, max_cmd)
        velocity_pub.publish(cmd)
        
        error_z = errors[2]
        rospy.loginfo(f"起飛中 | 高度: {current_pose.pose.position.z:.2f} m | 誤差: {error_z:.2f} | vz_cmd: {cmd.twist.linear.z:.2f}")
        
        if (abs(errors[0]) < 0.1 and abs(errors[1]) < 0.1 and abs(errors[2]) < 0.1 and
            abs(current_velocity.twist.linear.x) < 0.1 and 
            abs(current_velocity.twist.linear.y) < 0.1 and
            abs(current_velocity.twist.linear.z) < 0.1):
            rospy.loginfo(f"起飛完成，穩定在位置： {current_pose.pose.position.z:.2f} m。")
            break
        
        rate.sleep()
    
    rospy.loginfo("懸停 3 秒...")
    hover_start_time = rospy.Time.now()
    hover_duration = rospy.Duration(3.0)
    
    while not rospy.is_shutdown() and (rospy.Time.now() - hover_start_time) < hover_duration:
        # 持續發布心跳，避免有任何指令空檔 (會導致退出 OFFBOARD)
        zero_velocity_cmd.header.stamp = rospy.Time.now()
        velocity_pub.publish(zero_velocity_cmd) 
        rate.sleep()
    rospy.loginfo("懸停結束。")

    # 計算 B 點相對於起飛點的偏差
    target_b = [takeoff_point[0] + 18.0, takeoff_point[1], target_altitude]
    
    rospy.loginfo(f"移動至 B 點 ({target_b[0]:.2f}, {target_b[1]:.2f}, {target_b[2]:.2f})...")
    
    while not rospy.is_shutdown():
        cmd, errors = xyz_pd_controller(target_b, current_pose, current_velocity, kP, kD, max_cmd)
        velocity_pub.publish(cmd)
        
        # 計算到 B 點的三維距離
        distance_to_target = math.sqrt(errors[0]**2 + errors[1]**2 + errors[2]**2)
        
        rospy.loginfo(f"移動至 B 點 | 目前位置: ({current_pose.pose.position.x:.2f}, {current_pose.pose.position.y:.2f}, {current_pose.pose.position.z:.2f}) | 剩餘距離: {distance_to_target:.2f} m")
        
        # 檢查是否達到 B 點並且穩定
        if (distance_to_target < 0.2 and
            abs(current_velocity.twist.linear.x) < 0.1 and 
            abs(current_velocity.twist.linear.y) < 0.1 and
            abs(current_velocity.twist.linear.z) < 0.1):
            rospy.loginfo(f"已到達 B 點並穩定。")
            break
        rate.sleep()
    
    rospy.loginfo("在 B 點懸停 3 秒...")
    hover_start_time = rospy.Time.now()
    hover_duration = rospy.Duration(3.0)
    
    while not rospy.is_shutdown() and (rospy.Time.now() - hover_start_time) < hover_duration:
        zero_velocity_cmd.header.stamp = rospy.Time.now()
        velocity_pub.publish(zero_velocity_cmd) 
        rate.sleep()
    rospy.loginfo("B 點懸停結束，準備降落 !!!")
    
    # 發送約兩秒的速度指令以確保無人機有穩定滯空
    for _ in range(40):
        zero_velocity_cmd.header.stamp = rospy.Time.now()
        velocity_pub.publish(zero_velocity_cmd)
        rate.sleep()

    land_set_mode_req = SetModeRequest()
    land_set_mode_req.custom_mode = 'AUTO.LAND'

    last_req_time = rospy.Time.now()
    
    # 嘗試切換模式直到成功為止，而非只嘗試一次
    # 切換過程中持續發布心跳，避免被強制退出 OFFBOARD
    rospy.loginfo("嘗試切換至 AUTO.LAND 模式...")
    while not rospy.is_shutdown() and current_state.mode != "AUTO.LAND":
        if (rospy.Time.now() - last_req_time) > rospy.Duration(5.0):
            if set_mode_client.call(land_set_mode_req).mode_sent:
                rospy.loginfo("AUTO.LAND 模式請求已發送")
            last_req_time = rospy.Time.now()
        
        zero_velocity_cmd.header.stamp = rospy.Time.now()
        velocity_pub.publish(zero_velocity_cmd) 
        rate.sleep()

    while not rospy.is_shutdown() and current_state.armed:
        # 降落時不發布速度指令，讓 AUTO.LAND 完全接管
        rospy.loginfo(f"降落中... 高度: {current_pose.pose.position.z:.2f} m")
        rate.sleep()

    rospy.loginfo("--- 自主飛行結束 ---")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

