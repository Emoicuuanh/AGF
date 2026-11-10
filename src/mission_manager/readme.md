---
HuyNV
MKAC
May 05, 2021
---

# AGV Lidar Control center

## Các nhóm missions

- move
- charging
- pick_up_shelf
- place_shelf
- docking

## Các action

- `adjust_localization`: Không có tham số, chưa làm.
- `check_position_status`:
  - args:
    - `position`: pose(x, y, z)
    - bool `desired`: free or occupied
    - float `timeout`: thời gian chờ đợi kết quả ktra
  - return:
    - True: when at position as desired_result
    - False: when at position not as desired result

- `planner_setting`:
  - args:
    - `desired_speed`: Thiết lập vận tốc mong muốn cho mission
    - ...

- `move`:
  - args:
    - `goal`: pose(x, y, z) đích đến của robot
    - `retries`: Số lần thử nếu chưa đạt đc độ chính xác
    - `distance_threshold`: Khoảng cách cho phép sai số tại goal
  - result:
    - Succeeded
    - Cancel
    - Error

- `relative_move`: Di chuyển tới điểm dựa vào tọa độ tương đối, ứng dụng trong các trường hợp di chuyển vào sạc, chui vào xe.
  - args:
    - `X`: vị trí dọc trục robot, X dương phía trước rb, X âm sau robot
    - `Y`: vị trí theo phương vuông góc trục rb, Y dương bên phải, Y âm bên trái rb
    - `Orientation`: Góc sau khi robot di chuyển tới vị trí. Góc dương quay ngược chiều đồng hồ, góc âm quay cùng chiều đồng hồ.
    - `Max_linear_vel`: Vận tốc di chuyển tối đa
    - `Max_angular_vel`: Vận tốc góc tối đa

- `set_footprint`: Cập nhật lại footprint cho robot. Sử dụng để bỏ qua 4 chân của shelf hàng khi chở trên rb trong hàm safety. (Trong MIR dùng action này để cập nhật footprint phục vụ tránh vật cản)
  - args:
    - `footprint`: mảng tọa độ các đỉnh

- `docking`:
  - args:
    - `marker`: tên marker trạm sạc. Chứa thông tin: Vị trí sạc, loại marker
  - func:
    - Robot di chuyển về 1 điểm phía trước trạm sạc
    - Dùng ICP xác định chính xác vị trí sạc theo marker
    - Đk robot lùi về trạm sạc

- `charging`: action quản lý quá trình sạc. Cho phép sạc tới lúc đạt thời gian sạc tối thiểu, hoặc đạt % pin tối thiểu hoặc cho phép di chuyển khi có mission tiếp theo.
  - args:
    - `minimum_time`: Thời gian sạc tối thiểu
    - `minimum_percentage`: Phần trăm pin tối thiểu cần sạc.
    - `charge_until_new_mission_in_queue`: Cho phép robot rời trạm sạc khi có mission tiếp theo

- `pick_up_shelf`: Lấy shelf
  - args:
    - `marker_position`: Chọn vị trí marker
    - `marker_type`: Chọn loại marker (V, VL, bar, leg)
    - `self_footprint`: Chọn footprint của shelf để cập nhật
    - `safety_job`: Chọn vùng an toàn. ( -1 để tắt an toàn)
    - `undocking_distance`: Khoảng cách di chuyển ra khỏi vị trí để shelf. Giá trị dương là tiến, gía trị âm là lùi
  - func:
    - Di chuyển tới vị trí marker (xác định một điểm gần với vị trí shelf, dọc theo trục x)
    - Tìm vị trí chính xác của marker theo kiểu marker
    - Thay đổi safety_job
    - Di chuyển vào vị trí lấy shelf
    - Nâng bàn nâng lên để lấy shelf
    - Cập nhật footprint
    - Di chuyển tịnh tiến theo `undocking_distance` để ra khỏi vị trí đặt shelf (vì thường self được đặt trong không gian hẹp)
    - kết thúc action

- `place_shelf`: Thả shelf
  - args:
    - `undocking_distance`: Khoảng cách di chuyển ra khỏi shelf
  - func:
    - Start: Lúc rb đã di chuyển tới vị trí cần thả shelf
    - Hạ bàn nâng để thả shelf
    - Di chuyển tịnh tiến theo `undocking_distance` để ra khỏi shelf.
    - Cập nhật lại footprint của rb.
    - End action

| Action Result   | Mission behavior            |
| :---            | :---                        |
| SUCCEEDED       | Go on to next action        |
| PREEMPTED       | Cancel mission              |
| RETRY           | Retry current action        |
| ERROR           | Pause to check and fix Error|
| FAIL            | Cancel mission              |


## Cac buoc

- Step 1: Power on Robot. Program (`agvlidar_navigation.launch`) automatically run at startup.
- Step 2: Start Rviz to view and estimate initial pose
```
roslaunch agvlidar_navigation rviz.launch
````

- Step 3: Show scan to view scan safety
```
rosrun scan_safety show_scan.py
```

- Step 4: Start testing mission
```
rosrun mission_manager mission_client.py test_final.json
```