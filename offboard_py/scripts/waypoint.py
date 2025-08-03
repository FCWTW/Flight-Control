#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State, Waypoint
from mavros_msgs.srv import (CommandBool, SetMode, CommandBoolRequest, SetModeRequest,
                             WaypointPush, WaypointPushRequest, WaypointClear)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt16
import math

# 全域變數
current_state = None
current_global_pos = NavSatFix()
home_position = None
current_wp_index = -1

# 回呼函數
def state_callback(msg):
    global current_state
    current_state = msg

def global_pos_callback(msg):
    global current_global_pos
    current_global_pos = msg

def mission_current_callback(msg):
    global current_wp_index
    current_wp_index = msg.data

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
    
    if command == 16:  # MAV_CMD_NAV_WAYPOINT
        wp.param1 = hold_time
        wp.param4 = yaw
    elif command == 21:  # MAV_CMD_NAV_LAND
        wp.param4 = yaw
    elif command == 22: # MAV_CMD_NAV_TAKEOFF
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

def main():
    global current_state, current_global_pos, home_position, current_wp_index
    
    rospy.init_node("drone_waypoint_controller", anonymous=True)
    
    # 1. 初始化 ROS 訂閱者與服務客戶端
    rospy.Subscriber("/mavros/state", State, state_callback)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_pos_callback)
    rospy.Subscriber("/mavros/mission/current", UInt16, mission_current_callback)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    
    rospy.wait_for_service("/mavros/mission/push")
    mission_push_client = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
    
    rospy.wait_for_service("/mavros/mission/clear")
    mission_clear_client = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
    
    rate = rospy.Rate(20)
    
    # 2. 等待 MAVROS 連線並獲取起飛點 GPS
    rospy.loginfo("等待 MAVROS 連線...")
    while not rospy.is_shutdown() and not current_state:
        rate.sleep()
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("MAVROS 連線中...")
        rate.sleep()
    rospy.loginfo("✅ MAVROS 已連線。")
    
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
    rospy.loginfo(f"✅ 起飛點 GPS: ({home_position['lat']:.7f}, {home_position['lon']:.7f})")
    
    # 3. 清除現有任務
    rospy.loginfo("清除飛控上的舊任務...")
    try:
        mission_clear_client()
        rospy.loginfo("✅ 任務清除成功。")
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 任務清除失敗: {e}")
        return
    
    # 4. 創建航點清單 (起飛 -> 懸停 -> 飛到B點 -> 降落)
    waypoints = []
    
    takeoff_wp = create_waypoint(lat=home_position['lat'], lon=home_position['lon'], alt=2.0, command=22)
    waypoints.append(takeoff_wp)
    
    hover_wp = create_waypoint(lat=home_position['lat'], lon=home_position['lon'], alt=2.0, command=16, hold_time=5.0)
    waypoints.append(hover_wp)
    
    target_b_lat, target_b_lon = local_to_global_offset(0.0, 5.0, home_position['lat'], home_position['lon'])
    b_point_wp = create_waypoint(lat=target_b_lat, lon=target_b_lon, alt=2.0, command=16, hold_time=5.0)
    waypoints.append(b_point_wp)
    
    land_wp = create_waypoint(lat=target_b_lat, lon=target_b_lon, alt=0.0, command=21)
    waypoints.append(land_wp)

    # 5. 上傳任務
    rospy.loginfo("上傳任務航點...")
    try:
        req = WaypointPushRequest()
        req.start_index = 0
        req.waypoints = waypoints
        resp = mission_push_client(req)
        if resp.success:
            rospy.loginfo(f"✅ 任務上傳成功，共 {len(waypoints)} 個航點。")
            command_list = []
            for wp in waypoints:
                command_list.append(wp.command)
            rospy.loginfo(f"🚁 任務內容：{command_list}")
        else:
            rospy.logerr(f"❌ 任務上傳失敗，錯誤碼: {resp.error_code}")
            return
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 任務上傳服務異常: {e}")
        return
    
    # 6. 切換到 AUTO.MISSION 模式
    mission_set_mode_req = SetModeRequest()
    mission_set_mode_req.custom_mode = 'AUTO.MISSION'
    
    rospy.loginfo("嘗試切換至 AUTO.MISSION 模式...")
    last_req_time = rospy.Time.now()
    while not rospy.is_shutdown() and current_state.mode != "AUTO.MISSION":
        if (rospy.Time.now() - last_req_time) > rospy.Duration(5.0):
            if set_mode_client.call(mission_set_mode_req).mode_sent:
                rospy.loginfo("✅ AUTO.MISSION 模式請求已發送。")
            last_req_time = rospy.Time.now()
        rate.sleep()
    rospy.loginfo(f"🚁 目前模式：{current_state.mode}")
    
    # 7. 解鎖無人機並啟動任務
    arm_cmd_req = CommandBoolRequest()
    arm_cmd_req.value = True
    
    rospy.loginfo("嘗試解鎖無人機...")
    last_req_time = rospy.Time.now()
    while not rospy.is_shutdown() and not current_state.armed:
        if (rospy.Time.now() - last_req_time) > rospy.Duration(5.0):
            if arming_client.call(arm_cmd_req).success:
                rospy.loginfo("✅ 無人機解鎖請求已發送。")
            last_req_time = rospy.Time.now()
        rate.sleep()
    rospy.loginfo("✅ 無人機已解鎖，任務開始執行！")
    
    rospy.loginfo("=========================")
    rospy.loginfo("⚠️ 請自行觀察 Terminal 的 'WP: reached #X' 訊息")
    rospy.loginfo("=========================")
    rospy.sleep(5.0)
    
    # 8. 監控最終的降落過程，直到上鎖
    while not rospy.is_shutdown() and current_state.armed:
        rospy.sleep(1.0)
    
    rospy.loginfo("🎉 --- 自主飛行任務完成 ---")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass