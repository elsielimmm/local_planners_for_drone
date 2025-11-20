# Local Planners For Drone

## Tổng quan
Local Planners For Drone là hệ thống lập kế hoạch quỹ đạo cục bộ cho Drone trên nền ROS2 + Gazebo. Pipeline chính: cảm biến → bản đồ → tìm đường thô (A*) → tối ưu quỹ đạo (B‑spline) → xuất lệnh offboard cho PX4/MAVROS.

## Mục tiêu
- Sinh quỹ đạo an toàn, mượt cho Drone.    
- Tích hợp dữ liệu từ Gazebo (pointcloud / depth / odom) qua ros_gz_bridge.  
- Hỗ trợ replan động khi mục tiêu hoặc môi trường thay đổi.

## Thông tin dự án:
- Môn: Kỹ thuật Robot
- Giảng viên hướng dẫn: PGS. TS. Phan Trần Đăng Khoa
- Nhóm: 21.44
- Sinh viên thực hiện
+ Phạm Thị Phương - 106210050
+ Trần Thanh Tín - 106210253
+ Dương Thị Thảo Vi - 106210259
