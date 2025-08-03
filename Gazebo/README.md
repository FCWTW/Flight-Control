# Gazebo 虛擬環境修改

這是針對 Summer Course 所建立的基礎環境進行修改，PX4 是放在根目錄 (~/PX4-Autopilot)，同時已經建好 offboard package (~/catkin_ws/src/offboard_py)。

---
## 改善地板材質

1. 將[『asphalt.material』](https://github.com/FCWTW/Gazebo/blob/master/asphalt.material)複製到『~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/asphalt_plane/materials/scripts/asphalt.material』
2. 將『/usr/share/gazebo-11/media/materials/scripts/gazebo.material』中的 Gazebo/Grey 改成以下的樣子：
```python
material Gazebo/Grey
{
  technique
  {
    pass main
    {
      scene_blend alpha_blend
      depth_write off
      ambient 1.0 1.0 1.0 0.5
      diffuse 1.0 1.0 1.0 0.5
      specular 0.01 0.01 0.01 0.5 1.500000
    }
  }
}
```

---
## 幫無人機加裝 RGBD 相機

1. 將[『iris.sdf.jinja』](https://github.com/FCWTW/Gazebo/blob/master/iris.sdf.jinja)複製到『~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja』
(『iris.sdf.jinja』第 581 行開始是相機的設置，可以自行修改視野角度和可視距離)
2. 刪除相同資料夾下的『iris.sdf』
3. 執行以下指令重新生成無人機
```bash
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic_iris
```
4. 測試無人機是否有 RGBD 相機
```bash
roslaunch px4 posix_sitl.launch
```

Reference: [ViniciusAbrao/px4_ros2_xrcedds](https://github.com/ViniciusAbrao/px4_ros2_xrcedds?tab=readme-ov-file#rgbd-camera-simulation)、[Iris model with RGBD camera in Gazebo and ROS Noetic](https://www.youtube.com/watch?v=PpW_qEyGmyM)

---
## 簡易障礙物

1. 將[『empty.world』](https://github.com/FCWTW/Gazebo/blob/master/empty.world)複製到『~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world』
2. 新增以下資料夾並創建 picture.material
![image](https://hackmd.io/_uploads/rysjxOjLeg.png)
3. 將[『picture.material』](https://github.com/FCWTW/Gazebo/blob/master/materials/scripts/picture.material)複製下來，並使用[這裡的圖片](https://github.com/FCWTW/Gazebo/tree/master/materials/textures)。

Reference: [HKPolyU-UAV/E2ES](https://github.com/HKPolyU-UAV/E2ES)

---
## 整合 YOLO 與 offboard 程式

1. offboard package 有修改過的程式碼放在[這裡](https://github.com/FCWTW/Gazebo/tree/master/offboard_py)
2. 使用以下指令測試
```bash
# 測試無人機與相機
roslaunch offboard_py start_offb.launch

# 測試 YOLO 偵測
roslaunch offboard_py fly.launch
```

Reference: [YOLO Detect](https://hackmd.io/lwReki-hQTCrnzUMYMq2yA?view)