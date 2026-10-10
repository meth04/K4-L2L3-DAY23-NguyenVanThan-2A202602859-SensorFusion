# Ghi chú ôn vấn đáp

Đây là ghi chú ôn tập; học viên cần tự đọc code và giải thích bằng lời của mình.

## Một frame đi qua những bước nào?

Đọc Waymo → point cloud → BEV → detector → predict mỗi track đúng một lần →
associate/update LiDAR → associate/update camera nếu có FRONT → đánh giá.
Hai mode trong `compare` là hai lần chạy độc lập, không dùng chung trạng thái tracker.
Trong một lần chạy fused chỉ có một tracker; camera cập nhật chính các track đã được
LiDAR cập nhật, không tạo tracker riêng.

## Part E: công thức và kích thước

`x = [px, py, pz, vx, vy, vz]^T` có kích thước 6×1; `P`, `F`, `Q` là 6×6.
`F = [[I3, dt I3], [0, I3]]`; `dt = 0.1 s`.
Lab quy định `Q = dt*q*I6 = 0.3*I6`; không thay bằng mô hình gia tốc khác.

Predict: `x⁻ = Fx`, `P⁻ = FPFᵀ + Q`.
Update: `γ = z − h(x⁻)`, `S = HP⁻Hᵀ + R`,
`K = P⁻HᵀS⁻¹`, `x⁺ = x⁻ + Kγ`, `P⁺ = (I−KH)P⁻`.
LiDAR: `γ` 3×1, `H` 3×6, `S` 3×3, `K` 6×3.
Camera: `γ` 2×1, `H` 2×6, `S` 2×2, `K` 6×2.

## Part F: tại sao Mahalanobis?

`d² = γᵀS⁻¹γ` đo residual so với bất định. Ví dụ 1D:
residual 2 m với variance 4 m² cho `d²=1`, nhưng variance 0.01 m² cho `d²=400`.
Gating so sánh **bình phương** khoảng cách này với `chi2.ppf(0.995, dim_meas)`.
Greedy lấy cặp hữu hạn nhỏ nhất rồi bỏ cả hàng track và cột measurement;
mỗi đối tượng chỉ được dùng một lần. Greedy không đảm bảo tối ưu toàn cục như Hungarian.
FOV phải được kiểm tra trước phép chiếu/Mahalanobis để tránh chia cho depth không hợp lệ.
Frame rỗng vẫn phải gọi `manage_tracks(..., sensor)` để xử lý LiDAR miss.

## Part G: hệ tọa độ và Jacobian

Waymo camera có các trục forward/left/up, nên depth là `x_s`, không phải `z_s`.
`p_s = R_vehicle_to_sensor*p_vehicle + t_vehicle_to_sensor`.
`u = c_i − f_i*y_s/x_s`, `v = c_j − f_j*z_s/x_s`.
Từ chối tọa độ không hữu hạn hoặc `x_s <= 1e-6` trước khi chia.

Derivative theo vị trí sensor:

```text
          [ fi*ys/xs²   −fi/xs       0     ]
J_proj =  [ fj*zs/xs²     0       −fj/xs   ]
H = [ J_proj * R_vehicle_to_sensor | zeros(2,3) ]
```

Jacobian do platform tính; Part G cung cấp `h(x)` và FOV.
Camera `R=diag(25,25)` pixel²; LiDAR `R=diag(0.01,0.01,0.01)` m².
Pixel mô phỏng của runner thêm nhiễu std **0.5 px**, khác với std **5 px** mà
EKF giả định trong `R`. Đây là cấu hình giảng dạy, không phải noise được fit từ dataset.

## Part H: score và vòng đời

Track mới do LiDAR tạo: vị trí được đổi về hệ xe, vận tốc bằng 0,
score `1/6`, state `initialized`. LiDAR hit cộng `1/6`, tối đa 1;
LiDAR miss trong FOV trừ `1/6`. Camera không thay score/state hay tạo/xoá track.
`score > 0.8` thì confirmed: năm quan sát liên tiếp kể cả lần sinh track là đủ.
Confirmed không bị hạ thành tentative sau miss.
Xoá nếu `Pxx > 9` hoặc `Pyy > 9`, hoặc confirmed có `score < 0.6`,
hoặc chưa confirmed có `score <= 0`.
Ví dụ confirmed score 1: một miss → 5/6; hai miss → 4/6; ba miss → 3/6 và xoá.
Camera vẫn có thể ảnh hưởng gián tiếp lần gán LiDAR sau vì đã thay đổi `x/P`.

## Đọc metrics thế nào?

RMSE tính lỗi vị trí **3D** trên confirmed track ghép một-một với GT;
cổng ghép đánh giá dùng khoảng cách **XY <= 2 m**. Gate đánh giá này khác
gating Mahalanobis trong association.
`precision_track=matches/(matches+ghosts)`, `coverage=matches/det_tp`.
Không nhầm coverage với `matches/(matches+misses)` hoặc detection recall.
Hai mode có cùng số ghép không có nghĩa danh tính từng cặp luôn giống nhau.
Log chỉ chứa thống kê mỗi frame, không lưu mapping track–GT để chứng minh ID consistency.
Không suy ra nguyên nhân của từng miss từ tổng số liệu; cần trace theo frame nếu muốn kết luận.

## Giới hạn kết luận

Camera của lab lấy tâm hộp 2D GT, không chạy detector ảnh. Tâm hộp 2D cũng không nhất
thiết trùng ảnh chiếu của tâm hộp 3D. Kết quả fused tốt hơn trên một segment/seed
không chứng minh hiệu quả với camera detector thực hoặc mọi dữ liệu.
Thí nghiệm calibration giữ nguyên code tracking; chỉ thay extrinsic trong script bonus.
Các ảnh minh hoạ chọn những frame camera làm giảm lỗi để thấy tác dụng cụ thể;
đánh giá tổng thể phải dùng toàn bộ 199 frame, không chỉ ba ảnh được chọn.
