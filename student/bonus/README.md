# Bằng chứng bonus có thể tái lập

Hai phần đề nghị chấm: **trực quan hoá camera update (+3)** và **phân tích
calibration (+4)**. Chỉ dùng kết quả thực nghiệm sinh bằng script; không chỉnh tay
metrics/log. Không yêu cầu điểm bonus CVAT: chưa có môi trường để import và chụp
ảnh xác minh. `tracks_export.json` chỉ là output của helper export có sẵn, không
được coi là bằng chứng import CVAT.

## Chạy lại

Cài môi trường theo README gốc, tải segment/weights mặc định, tạo
`student/config/paths.yaml` từ file mẫu với `frame_start: 0`, `frame_end: 198`.
Từ gốc repo trên Windows:

```powershell
.\.venv\Scripts\python.exe -X utf8 -B student/bonus/run_experiments.py
.\.venv\Scripts\python.exe -X utf8 -B -m fusion_lab.scripts.run_lab --config student/config/paths.yaml --fusion compare --seed 0
.\.venv\Scripts\python.exe -X utf8 -B student/bonus/verify_evidence.py
.\.venv\Scripts\python.exe -X utf8 -B -m pytest student/tests -q
.\.venv\Scripts\python.exe -X utf8 -B tools/check_submission.py
```

Trên Linux/macOS dùng `.venv/bin/python` thay đường dẫn Windows. UTF-8 cần thiết
trên Windows vì một số fixture của bộ test ghi tiếng Việt mà không chỉ định
encoding. Script bonus dùng 4 CPU threads; có thể có sai khác làm tròn rất nhỏ
so với runner dùng số thread mặc định.

## Thiết kế thí nghiệm

`run_experiments.py` chạy detector gốc một lần mỗi frame, rồi dùng cùng detection
cho sáu tracker độc lập: LiDAR và fused với extrinsic camera lệch yaw
`0°, 0.5°, 2°, 5°, 10°`. Tất cả xử lý đủ 199 frame, seed 0, cùng cấu hình tracking.
Mỗi tracker có RNG riêng cùng seed nên nhận cùng chuỗi pixel nhiễu. Chỉ thay
transform mà tracker **giả định**: `T_bad = T_true @ Rz(yaw)` quanh trục up của camera.
Giữ nguyên nhãn vật lý, nội tại camera, detector, noise covariance và thresholds.
Không sửa file platform hoặc artifacts chấm điểm khi chạy bonus.

Đo hai loại bằng chứng:

- **Tracking thực tế**: `metrics_yaw_*.json`, `records_yaw_*.jsonl` và
  `camera_updates_yaw_*.csv` chứa kết quả full replay và residual các update được
  chấp nhận. Mỗi metrics được validate lại từ toàn bộ records.
- **Probe hình học cố định**: tại mỗi cặp được camera nominal chấp nhận, đóng băng
  cùng `x/P/z` trước update rồi tính residual và `d²` cho từng extrinsic. Bảng
  `calibration_probes.csv` loại ảnh hưởng của quỹ đạo khác nhau và selection bias
  khi chỉ quan sát các update được gating chấp nhận. Probe không cập nhật tracker.
  Đây là cặp tham chiếu do nominal association chọn, không phải ground-truth
  correspondence độc lập.

`calibration_summary.csv/json` tổng hợp hai loại đo. `probe_gate_pass_fraction`
tính trên toàn bộ cặp nominal, kể cả cặp bị FOV loại; `probe_visible_pairs` cho
biết số cặp còn trong FOV. Gate camera là `chi2.ppf(0.995, 2) ≈ 10.5966`.
RMSE có thể không tăng đơn điệu khi lệch calibration: khi residual quá lớn,
gating loại camera update và tracker quay gần về chất lượng LiDAR.

## Ảnh và dữ liệu đi kèm

Ba ảnh `camera_update_*.png` được chọn từ ba đoạn thời gian khác nhau của segment.
Mỗi ảnh có BEV, vùng phóng to cùng một track trước/sau camera update, và ảnh FRONT
với pixel dự đoán/quan sát. Các marker màu: xanh lá = tâm GT 3D; cam = trước camera;
xanh dương = sau camera; hồng = pixel camera mô phỏng có nhiễu. Tiêu đề ghi lỗi
vị trí 3D trước/sau theo mét và frame/track ID. Score/state và tập ID được assert
không đổi qua camera pass trong mọi frame.

`visualization_evidence.json` lưu toạ độ và lỗi tương ứng để kiểm tra lại các nhãn
trên ảnh. Ảnh chọn các trường hợp giảm lỗi nhằm minh hoạ tác dụng; không dùng các
ảnh chọn lọc để kết luận chất lượng toàn segment. `calibration.png` trình bày
full-run RMSE, mean horizontal residual trên probe và tỷ lệ probe vượt gating.

`provenance.json` ghi segment, seed, phạm vi frame và SHA-256 raw bytes của code
E–H, weights, script tại máy chạy. Digest raw bytes có thể khác khi Git chuyển
LF/CRLF trên hệ điều hành khác; số liệu và công thức không phụ thuộc line endings.
`verify_evidence.py` kiểm tra logs/metrics, tổng hợp CSV, tái lập nominal so với
runner gốc và lỗi trên ảnh, chỉ đọc dữ liệu, không ghi lại kết quả.

Xem bảng số liệu và diễn giải cuối cùng trong [báo cáo](../SUBMISSION.md), và
[ghi chú vấn đáp](VAN_DAP.md) để ôn lại các công thức và giới hạn kết luận.
