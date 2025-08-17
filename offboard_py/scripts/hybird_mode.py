#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State, Waypoint
from mavros_msgs.srv import (CommandBool, SetMode, CommandBoolRequest, SetModeRequest,
                             WaypointPush, WaypointPushRequest, WaypointClear)
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import UInt16, String
import math

# 全域變數
current_state = None
current_global_pos = NavSatFix()
home_position = None
current_wp_index = -1   # 這個修不好，先不用
current_pose = PoseStamped()
current_velocity = TwistStamped()
heartbeat = TwistStamped()   # For OFFBOARD
heartbeat.header.frame_id = 'base_link'
key_cmd = "stop"

TIMEOUT_DURATION = 5.0
TAKEOFF_ALTITUDE = 3.0
AVOIDANCE_DURATION = 4.0

# AUTO.MISSION 的航點目標 (B點)
b_lat = 0.0
b_lon = 20.0

# Callback function
def state_callback(msg):
    global current_state
    current_state = msg

def global_pos_callback(msg):
    global current_global_pos
    current_global_pos = msg

def mission_current_callback(msg):
    global current_wp_index
    current_wp_index = msg.data

def pose_callback(msg):
    global current_pose
    current_pose = msg

def velocity_callback(msg):
    global current_velocity
    current_velocity = msg

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def key_cb(msg):
    global key_cmd, last_cmd_time
    cmd = msg.data.lower().strip()
    if cmd in ["forward", "backward", "left", "right", "stop"]:
        key_cmd = cmd
        last_cmd_time = rospy.Time.now()
    else:
        rospy.logwarn(f"未知指令：{cmd}")

def create_waypoint(lat, lon, alt, command, hold_time=0.0, yaw=float('nan')):
    """
    創建一個 MAVROS Waypoint 物件。
    參數:
      - lat, lon, alt: 目標緯度、經度、高度。
      - command: MAV_CMD_... 指令代碼。
      - hold_time: 懸停時間（秒），僅適用於某些指令。
      - yaw: 目標偏航角（度）。
    """
    wp = Waypoint()
    wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp.command = command
    wp.is_current = False
    wp.autocontinue = True
    
    if command == 16:  # 飛行，MAV_CMD_NAV_WAYPOINT
        wp.param1 = hold_time
        wp.param4 = yaw
    elif command == 21:  # 降落，MAV_CMD_NAV_LAND
        wp.param4 = yaw
    elif command == 22: # 起飛，MAV_CMD_NAV_TAKEOFF
        wp.param1 = 0.0
        wp.param4 = yaw

    wp.x_lat = lat
    wp.y_long = lon
    wp.z_alt = alt
    return wp

def local_to_global_offset(local_x, local_y, home_lat, home_lon):
    """
    將本地座標偏移轉換為 GPS 座標偏移，使用更精確的地球模型。
    """
    a = 6378137.0
    e2 = 6.69437999014e-3
    
    lat_rad = math.radians(home_lat)
    lon_rad = math.radians(home_lon)
    
    N = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
    M = a * (1 - e2) / math.sqrt((1 - e2 * math.sin(lat_rad)**2)**3)
    
    dlat = local_x / M
    dlon = local_y / (N * math.cos(lat_rad))
    
    target_lat = home_lat + math.degrees(dlat)
    target_lon = home_lon + math.degrees(dlon)
    
    return target_lat, target_lon

def pd_controller(target, current_pos, current_vel, kP, kD, max_cmd, dead_zone=0.03):
    """
    PD 控制器
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
        # 速度也很小時才真正停止控制
        if abs(current_vel) < 0.05:
            cmd = 0.0
        else:
            cmd = -kD * current_vel
        cmd = clamp(cmd, -max_cmd, max_cmd)
        return cmd, error
    cmd = kP * error - kD * current_vel
    if abs(cmd) > max_cmd:
        # 使用 sigmoid 函數平滑限制，而非硬切斷
        cmd = max_cmd * (cmd / abs(cmd)) * (1 - math.exp(-abs(cmd)/max_cmd))
    cmd = clamp(cmd, -max_cmd, max_cmd)
    return cmd, error

def set_mode(target, client, rate, velocity_pub=None):
    """
    切換無人機的模式
    """
    global current_state
    last_req_time = rospy.Time.now()
    rospy.loginfo(f"⚠️  嘗試切換至 {target} 模式...")
    while not rospy.is_shutdown() and current_state.mode != target:
        if (rospy.Time.now() - last_req_time) > rospy.Duration(TIMEOUT_DURATION):
            if client.call(custom_mode=target).mode_sent:
                rospy.loginfo(f"{target} 模式請求已發送。")
            last_req_time = rospy.Time.now()
        if velocity_pub != None:
            heartbeat.header.stamp = rospy.Time.now()
            velocity_pub.publish(heartbeat) 
        rate.sleep()
    rospy.loginfo(f"🚁 目前模式：{current_state.mode}")

def upload_mission(waypoints, push_client, clear_client):
    """
    清除舊任務並上傳新任務
    """
    # rospy.loginfo("清除飛控上的舊任務...")
    try:
        clear_client()
        # rospy.loginfo("✅ 任務清除成功。")
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 任務清除失敗: {e}")
        return False
    
    # rospy.loginfo("上傳任務航點...")
    try:
        req = WaypointPushRequest()
        req.start_index = 0
        req.waypoints = waypoints
        resp = push_client(req)
        if resp.success:
            rospy.loginfo(f"✅ 任務上傳成功，共 {len(waypoints)} 個航點。")
            return True
        else:
            rospy.logerr(f"❌ 任務上傳失敗，錯誤碼: {resp.error_code}")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 任務上傳服務異常: {e}")
        return False

def offboard_avoidance(velocity_pub, rate, height):
    """
    進入 OFFBOARD 模式並執行模擬避障
    """
    # 設置 Offboard PD 控制參數
    kP = [0.4, 0.4, 1.2]
    kD = [0.08, 0.08, 0.4]
    max_cmd = [0.5, 0.5, 0.5]
    
    # 模擬避障，先向右 3 秒，再向前 3 秒，全程定高飛行
    last_req_time = rospy.Time.now()
    cmd = TwistStamped()
    cmd.header.frame_id = 'base_link'
    
    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - last_req_time).to_sec()
        
        # 垂直高度控制
        vz_cmd, _ = pd_controller(height, current_pose.pose.position.z,
                                  current_velocity.twist.linear.z,
                                  kP[2], kD[2], max_cmd[2], 0.03)
        cmd.twist.linear.z = vz_cmd
        
        if elapsed_time < AVOIDANCE_DURATION:
            cmd.twist.linear.x = 0.0
            cmd.twist.linear.y = 0.5
        elif elapsed_time < AVOIDANCE_DURATION*2:
            cmd.twist.linear.x = 0.5
            cmd.twist.linear.y = 0.0
        else:
            # 避障完成，發送零速度指令並結束
            heartbeat.header.stamp = rospy.Time.now()
            velocity_pub.publish(heartbeat)
            break
        
        cmd.header.stamp = rospy.Time.now()
        velocity_pub.publish(cmd)
        rate.sleep()

def main():
    global current_state, current_global_pos, home_position, current_wp_index, current_pose, current_velocity
    
    rospy.init_node("drone_waypoint_controller", anonymous=True)
    
    # 初始化 ROS Subscriber & service client
    rospy.Subscriber("/mavros/state", State, state_callback)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_pos_callback)
    rospy.Subscriber("/mavros/mission/current", UInt16, mission_current_callback)
    rospy.Subscriber("/drone/keyboard_cmd", String, key_cb)
    velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    
    rospy.wait_for_service("/mavros/mission/push")
    mission_push_client = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
    
    rospy.wait_for_service("/mavros/mission/clear")
    mission_clear_client = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
    
    rate = rospy.Rate(20)
    
    # 等待 MAVROS 連線
    rospy.loginfo("等待 MAVROS 連線...")
    while not rospy.is_shutdown() and not current_state:
        rate.sleep()
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("MAVROS 連線中...")
        rate.sleep()
    rospy.loginfo("✅ MAVROS 已連線。")
    
    # 獲取起飛點 GPS
    rospy.loginfo("等待 GPS 定位...")
    gps_timeout = rospy.Time.now() + rospy.Duration(30.0)
    while not rospy.is_shutdown() and (current_global_pos.latitude == 0.0 or current_global_pos.longitude == 0.0):
        if rospy.Time.now() > gps_timeout:
            rospy.logwarn("⚠️ GPS 定位超時，將使用預設座標進行測試。")
            current_global_pos.latitude = 47.3977508
            current_global_pos.longitude = 8.5455947
            current_global_pos.altitude = 488.0
            break
        rate.sleep()
    home_position = {
        'lat': current_global_pos.latitude,
        'lon': current_global_pos.longitude,
        'alt': current_global_pos.altitude
    }
    height = current_pose.pose.position.z + TAKEOFF_ALTITUDE
    rospy.loginfo(f"✅ 起飛點 GPS: ({home_position['lat']:.7f}, {home_position['lon']:.7f})")
    
    # 創建航點清單 (起飛 -> 懸停 -> 飛到B點懸停 -> 降落)
    waypoints = []
    
    takeoff_wp = create_waypoint(lat=home_position['lat'], lon=home_position['lon'], alt=TAKEOFF_ALTITUDE, command=22)
    waypoints.append(takeoff_wp)
    
    hover_wp = create_waypoint(lat=home_position['lat'], lon=home_position['lon'], alt=TAKEOFF_ALTITUDE, command=16, hold_time=5.0)
    waypoints.append(hover_wp)
    
    target_b_lat, target_b_lon = local_to_global_offset(b_lat, b_lon, home_position['lat'], home_position['lon'])
    b_point_wp = create_waypoint(lat=target_b_lat, lon=target_b_lon, alt=TAKEOFF_ALTITUDE, command=16, hold_time=5.0)
    waypoints.append(b_point_wp)
    
    land_wp = create_waypoint(lat=target_b_lat, lon=target_b_lon, alt=0.0, command=21)
    waypoints.append(land_wp)

    # 上傳任務
    if not upload_mission(waypoints, mission_push_client, mission_clear_client):
        rospy.logerr("⛔ 初始任務上傳失敗，程式終止。")
        return
    
    # 切換到 AUTO.MISSION 模式
    # MPC_XY_VEL_ALL 要調整，不然會飛太快
    set_mode("AUTO.MISSION", set_mode_client, rate)
    
    # 解鎖無人機
    arm_cmd_req = CommandBoolRequest()
    arm_cmd_req.value = True
    rospy.loginfo("嘗試解鎖無人機...")
    last_req_time = rospy.Time.now()
    while not rospy.is_shutdown() and not current_state.armed:
        if (rospy.Time.now() - last_req_time) > rospy.Duration(TIMEOUT_DURATION):
            if arming_client.call(arm_cmd_req).success:
                rospy.loginfo("✅ 無人機解鎖請求已發送。")
            last_req_time = rospy.Time.now()
        rate.sleep()
    rospy.loginfo("✅ 無人機已解鎖，任務開始執行！")
    
    # 等待任務完成
    while not rospy.is_shutdown() and current_state.armed:
        if key_cmd != "stop":
            rospy.loginfo(f"⚠️⚠️⚠️⚠️⚠️  偵測到鍵盤輸入")
            # 切換至 OFFBOARD 準備避障
            set_mode("OFFBOARD", set_mode_client, rate, velocity_pub)

            # 模擬避障，先向右 4 秒，再向前 8 秒，全程定高飛行
            rospy.loginfo(f"⚠️  模擬避障中...")
            offboard_avoidance(velocity_pub, rate, height)

            # 在空中重新規劃任務 (飛到B點懸停 -> 降落)
            waypoints = []
            b_point_wp = create_waypoint(lat=target_b_lat, lon=target_b_lon, alt=2.0, command=16, hold_time=5.0)
            waypoints.append(b_point_wp)
            land_wp = create_waypoint(lat=target_b_lat, lon=target_b_lon, alt=0.0, command=21)
            waypoints.append(land_wp)

            # 過程中有任何錯誤會直接切換至 AUTO.LAND
            rospy.loginfo(f"⚠️  重新規劃任務...")
            if not upload_mission(waypoints, mission_push_client, mission_clear_client):
                rospy.logerr("⛔ 重新規劃任務失敗！")
                set_mode("AUTO.LAND", set_mode_client, rate, velocity_pub)

            # 發送約 2 秒的速度指令以確保無人機有穩定滯空
            for _ in range(40): 
                heartbeat.header.stamp = rospy.Time.now()
                velocity_pub.publish(heartbeat)
                rate.sleep()

            # 從 OFFBOARD 切回 AUTO.MISSION，繼續飛向 B 點
            set_mode("AUTO.MISSION", set_mode_client, rate, velocity_pub)

        rospy.sleep(0.1)
    
    rospy.loginfo("--- 自主飛行任務完成 ---")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass