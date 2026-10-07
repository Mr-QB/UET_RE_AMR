# Chạy bringup và chuyển mode SLAM/Nav2

Bringup được chạy một lần. Các lệnh `ros2 service call` gửi yêu cầu lưu map hoặc chuyển mode; không cần chạy launch file mode riêng.

## Build và source

```bash
cd ~/UET_RE_AMR/ros2
colcon build --packages-up-to uet_amr_bringup
source install/setup.bash
```

Sau khi mở terminal mới, source lại `install/setup.bash`.

## Terminal 1: chạy bringup

```bash
cd ~/UET_RE_AMR/ros2
source install/setup.bash
ros2 launch uet_amr_bringup amr_bringup.launch.py mode:=slam
```

Để terminal này chạy. Có thể bật RViz bằng `rviz:=true`.

## Terminal 2: gọi service

```bash
cd ~/UET_RE_AMR/ros2
source install/setup.bash
ros2 service type /amr/set_mode
```

Kiểu service mong đợi: `uet_amr_bringup/srv/SetMode`.

### Lưu map, tiếp tục SLAM

Service gọi `uet_amr_navigation/save_map.launch.py` để lưu occupancy map và serialize pose graph tuần tự. Mỗi map nằm trong thư mục riêng dưới `~/UET_RE_AMR/ros2/src/uet_amr_navigation/maps`. Ví dụ map `school` có các file:

```text
~/UET_RE_AMR/ros2/src/uet_amr_navigation/maps/school/school.yaml
~/UET_RE_AMR/ros2/src/uet_amr_navigation/maps/school/school.pgm
~/UET_RE_AMR/ros2/src/uet_amr_navigation/maps/school/school.posegraph
~/UET_RE_AMR/ros2/src/uet_amr_navigation/maps/school/school.data
```

```bash
ros2 service call /amr/set_mode uet_amr_bringup/srv/SetMode \
  "{mode: slam, map_yaml: '', save_current_map: true, map_name: warehouse}"
```

Response gồm đường dẫn đến `warehouse.yaml`. Khi request vừa lưu map vừa chuyển sang Nav2, manager tự truyền YAML vừa lưu cho Nav2.

### Lưu map rồi chuyển sang Nav2

```bash
ros2 service call /amr/set_mode uet_amr_bringup/srv/SetMode \
  "{mode: nav, map_yaml: '', save_current_map: true, map_name: warehouse}"
```

### Chuyển sang Nav2 với map đã lưu

```bash
ros2 service call /amr/set_mode uet_amr_bringup/srv/SetMode \
  "{mode: nav, map_yaml: '$HOME/UET_RE_AMR/ros2/src/uet_amr_navigation/maps/warehouse/warehouse.yaml', save_current_map: false, map_name: ''}"
```

Shell sẽ thay `$HOME` bằng home directory trên máy chạy ROS.

### Chuyển từ Nav2 về SLAM

```bash
ros2 service call /amr/set_mode uet_amr_bringup/srv/SetMode \
  "{mode: slam, map_yaml: '', save_current_map: false, map_name: ''}"
```

Đợi service trả response trước khi gửi yêu cầu chuyển mode tiếp theo. Không chạy `slam.launch.py` hoặc `navigation.launch.py` riêng khi mode manager đang quản lý chúng.

## Khởi động trực tiếp ở Nav2

```bash
ros2 launch uet_amr_bringup amr_bringup.launch.py \
  mode:=nav map:=$HOME/UET_RE_AMR/ros2/src/uet_amr_navigation/maps/warehouse/warehouse.yaml
```
