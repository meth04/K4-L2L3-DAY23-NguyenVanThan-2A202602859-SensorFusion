# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Báo cáo phần bắt buộc và hai thí nghiệm bonus. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Văn Thân
- MSSV: 2A202602859
- Email: 26ai.thannv2@vinuni.edu.vn
- Link repo (fork): https://github.com/meth04/K4-L2L3-DAY23-NguyenVanThan-2A202602859-SensorFusion
- Commit code/artifacts/bonus đã kiểm thử: `76f498225ae02d1ca48a49461a440e0125114cf2`.
- Commit hash nộp (`git rev-parse HEAD`): commit `CP6` cuối cùng tại
  [HEAD trên main](https://github.com/meth04/K4-L2L3-DAY23-NguyenVanThan-2A202602859-SensorFusion/commits/main);
  nộp hash đầy đủ của commit này cùng link repo trên LMS. Commit `CP6` chỉ hoàn tất
  báo cáo; code và bằng chứng giữ nguyên từ commit kiểm thử ở trên.

## Tóm tắt kết quả

- Cấu hình lần chạy chấm điểm (`fusion_mode`, `frames`, `segment`, `seed`): `compare`,
  `[0, 198]` (199 frame),
  `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`,
  `seed = 0`.
- Chất lượng phát hiện (`detection.precision`, `detection.recall`, `detection.tp/fp/fn`):
  `precision = 0.9701`, `recall = 0.7004`, `tp = 519`, `fp = 16`, `fn = 222`.
- Bám bắt ở chế độ lidar (`tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`):
  `rmse = 0.15028 m`, `matches = 502`, `sum_sq_err = 11.33692`,
  `ghost_track_frames = 0`, `missed_gt_frames = 239`,
  `mean_confirmed_tracks = 2.5226`.
- Bám bắt ở chế độ fused (`tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`):
  `rmse = 0.13585 m`, `matches = 502`, `sum_sq_err = 9.26422`,
  `ghost_track_frames = 0`, `missed_gt_frames = 239`,
  `mean_confirmed_tracks = 2.5226`.
- Phân tích so sánh hai chế độ dựa trên RMSE, số cặp ghép và thống kê ghost/miss:
  Hai chế độ có `det_tp/fp/fn` trùng khớp theo từng frame, cùng tổng `matches = 502`,
  `ghosts = 0`, `misses = 239` và `mean_confirmed_tracks`. LiDAR trực tiếp quyết định
  khởi tạo, score, xác nhận và xoá track; camera chỉ hiệu chỉnh `x/P`. Tuy nhiên,
  camera có thể ảnh hưởng gián tiếp phép gán LiDAR ở frame sau qua `x/P`; các tổng
  bằng nhau không chứng minh danh tính từng cặp ghép không đổi. Ở lần chạy này, `rmse`
  của fused (`0.13585 m`) nhỏ hơn `rmse` của lidar (`0.15028 m`), tương ứng mức giảm
  `0.01443 m` (khoảng `9.60%`). Chế độ fused có thêm lượt camera update (AssocC) khi
  FRONT có quan sát và cặp ghép vượt qua FOV/gating, tinh chỉnh vị trí trên cùng các
  track. Cả hai giá trị
  đều nằm dưới ngưỡng `0.45 m` (đạt điểm tối đa phần RMSE), và chênh lệch
  `fused − lidar = −0.01443 m ≤ 0.05 m` đạt tiêu chí fusion nhất quán. Số track ma
  bằng 0. Tổng `239` tracking misses lớn hơn `222` detection FN là `17`, nhưng không
  thể xác định nguyên nhân của từng miss hoặc loại trừ xoá nhầm chỉ từ các tổng này.
  Thời gian chờ xác nhận track và detector bỏ sót đều có thể góp phần; muốn phân loại
  chính xác cần trace detection/track/GT theo frame.

| Chỉ số dùng trong rubric | LiDAR | Fused | Ngưỡng tối đa |
|---|---:|---:|---:|
| RMSE 3D (m) | 0.150278 | 0.135848 | ≤ 0.45 |
| `precision_track = matches/(matches+ghosts)` | 1.000000 | 1.000000 | ≥ 0.75 |
| `coverage = matches/det_tp` | 0.967245 | 0.967245 | ≥ 0.70 |
| Hệ số chất lượng `q` | 1 | 1 | 1 |

Theo công thức trong [RUBRIC.md](../RUBRIC.md), các số liệu này đáp ứng mức tối đa
`20 + 15 + 10 = 45` điểm tự động B nếu cả bốn Part vượt bộ test gốc; A đáp ứng các
điều kiện tính nhất quán của log/metrics. Đây là đối chiếu tiêu chí, không phải điểm
giảng viên đã chấm. Phần thủ công, báo cáo và vấn đáp vẫn cần được đánh giá.

Lệnh chạy được thực thi từ thư mục gốc của repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

Chỉ số `rmse = sqrt(sum_sq_err/matches)` được tính trên vị trí 3D của các confirmed
track ghép một-một với GT xe trong cửa sổ BEV, với gate XY **2.0 m**; giá trị `null`
khi không tồn tại cặp ghép. Camera sử dụng tâm hộp 2D ground-truth FRONT có nhiễu theo
seed, **không** dùng camera detector. Do đó kết quả này không đo hiệu năng của một
perception system độc lập với ground truth.

Runner thêm nhiễu Gaussian có std `0.5 px` vào tâm hộp FRONT; EKF dùng
`sigma_cam_i = sigma_cam_j = 5 px`, tức `R = diag(25,25)`. Đây là hai tham số khác
nhau: nhiễu tạo observation và mức bất định giả định khi update. Tâm hộp 2D cũng
không nhất thiết trùng phép chiếu của tâm hộp 3D, nên residual không chỉ chứa nhiễu.

`grade_run.log` ở định dạng JSONL, mỗi cặp `(mode, frame)` ứng với đúng một record gồm
các trường: `mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`,
`matches`, `sum_sq_err`, `ghosts`, `misses`. Các ràng buộc `matches+ghosts==confirmed`
và `matches+misses==valid_gt` đều được thoả; tổng/trung bình trên các record khớp với
`metrics.json`. Các file theo từng chế độ `metrics_lidar.json`, `metrics_fused.json`,
`grade_run_lidar.log`, `grade_run_fused.log` được giữ lại để đối chiếu.

Ví dụ record LiDAR ở frame 186: `det_tp=3`, `det_fp=0`, `det_fn=9`, `valid_gt=12`,
`confirmed=3`, `matches=3`, `ghosts=0`, `misses=9`. Frame này có cả 9 detection FN
và 9 tracking misses: bộ phát hiện chỉ tìm được 3/12 GT, còn tracker ghép được cả
3 confirmed track với GT. Hai loại miss trùng số ở frame này nhưng khác định nghĩa;
không được suy ra chúng luôn tương đương hoặc cùng danh tính trên toàn segment.

## Giải thích ngắn (Parts E–H — tự viết)

1. **Khác biệt giữa quan sát lidar 3D và camera 2D trong EKF (`z`, `R`)?**
   Lidar cung cấp quan sát vị trí 3D `z = (x, y, z)` trong hệ xe, nên toán tử `H` là
   tuyến tính (khối 3×3 hằng số, các cột ứng với vận tốc bằng 0), `dim_meas = 3` và
   `R = diag(0.1², 0.1², 0.1²)` (m²). Camera cung cấp quan sát 2D theo pixel
   `z = (u, v)`, `dim_meas = 2`, `R = diag(5², 5²)` (pixel²); do `h(x)` là phép chiếu
   pinhole phi tuyến nên `H` phải là Jacobian `∂h/∂x` tại trạng thái hiện thời (platform
   tính bằng chain rule). Vì vậy cùng một bộ lọc EKF nhưng `z`, `R`, `H` và số chiều
   của innovation khác nhau giữa hai modality.
2. **Vì sao cần gating Mahalanobis trước khi thực hiện gán cặp?**
   Khoảng cách Mahalanobis `d² = γᵀ S⁻¹ γ` chuẩn hoá độ lệch theo bất định của track
   (`P`) và nhiễu quan sát (`R`), nên một độ lệch vài mét ở track có `P` lớn vẫn có thể
   được coi là hợp lý, trong khi một độ lệch nhỏ ở track có phương sai nhỏ lại là bất
   thường — khác với khoảng cách Euclidean thuần. Cổng χ²
   (`d² ≤ χ²_{0.995, dim_meas}`) loại các cặp vô lý (phát hiện nhầm, track đã trôi)
   trước khi gán, qua đó tránh ghép sai làm hỏng trạng thái track.
3. **Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log.**
   Detection tạo hộp xe trong từng frame; tracking nối quan sát theo thời gian,
   duy trì ID và ước lượng cả vị trí/vận tốc cùng covariance, kể cả khi quan sát
   tạm thời mất. Một detection mới không tự động là confirmed track.
   Pipeline là **track-then-fuse**: trong mỗi lần chạy tồn tại một tracker; mỗi frame thực hiện
   `predict` đúng một lần, sau đó lần lượt cập nhật lidar (AssocL) rồi camera (AssocC)
   trên cùng danh sách track. Bằng chứng trực tiếp là hàm
   [`run()` trong platform](../platform/fusion_lab/scripts/run_lab.py): vòng
   `KF.predict(track)` đứng trước `associate_and_update(..., lidar_sensor)` rồi
   `associate_and_update(..., camera_sensor)`. Trong log compare có **398 record**:
   một record cho mỗi `(mode, frame)`, với 199 frame ở mỗi mode. Ví dụ frame 0 có
   cả record `lidar` và `fused`; detection counts trùng nhau. Log xác nhận thống kê
   đầu ra hai mode; thứ tự gọi trong code mới chứng minh trình tự predict/update.
   `run_compare()` gọi hai lần `run()` độc lập nên hai mode không chia sẻ trạng thái.
4. **Nếu camera lệch calibration, innovation/residual sẽ biểu hiện ra sao?**
   Khi extrinsic bị lệch, `h(x)` chiếu trạng thái track ra sai toạ độ pixel, do đó
   innovation `γ = z − h(x)` xuất hiện **sai lệch hệ thống** (bias) theo một hướng thay
   vì chỉ dao động ngẫu nhiên quanh 0; `S = H P Hᵀ + R` và gain `K` vẫn được tính như
   bình thường nhưng kéo trạng thái về phía sai. Nếu độ lệch đủ lớn, `d²` vượt ngưỡng
   χ² nên cặp bị **gating loại** (camera không còn cập nhật track tương ứng); nếu vẫn
   lọt qua cổng, trạng thái fused bị kéo lệch khỏi lidar. Đây là lý do calibration cần
   khớp để chế độ fused không suy giảm so với lidar.
5. **Vì sao cần truyền `sensor` tường minh ở frame rỗng, và vì sao lidar quyết định còn camera chỉ cập nhật?**
   Khi `meas_list` rỗng, không thể suy ra sensor từ bất kỳ phần tử nào, nhưng vòng đời
   track vẫn phải được duy trì: những track không ghép được nhưng nằm trong FOV lidar
   phải bị trừ điểm tồn tại (miss). Vì vậy
   `associate_and_update(manager, [], filter, sensor)` nhận `sensor` tường minh và luôn
   gọi `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)` ở cuối mỗi
   lượt, kể cả khi danh sách rỗng. Chỉ lidar quyết định sự tồn tại của track vì lidar
   cung cấp quan sát 3D đáng tin cậy về vị trí và kích thước để khởi tạo và xác nhận
   track; camera chỉ tinh chỉnh trạng thái EKF (giữ nguyên score/state), không cộng hay
   trừ điểm, không khởi tạo và không xoá track.
6. **Điều kiện xác nhận, duy trì trạng thái confirmed sau miss và điều kiện xoá track.**
   Ở mỗi lượt lidar: một hit cộng `1/window` (tối đa bằng 1), một miss trong FOV lidar
   trừ `1/window`. Track được xác nhận khi `score > confirmed_threshold` (0.8); track
   đã confirmed **không** bị hạ trạng thái chỉ vì một lần miss. Track bị xoá khi thoả
   **một trong các** điều kiện: `P[0,0]` hoặc `P[1,1]` lớn hơn `max_P` (9.0); track đã
   confirmed có `score < delete_threshold` (0.6); hoặc track chưa confirmed có
   `score <= 0`.

Bằng chứng code cho các câu trả lời:
[`kalman.py`](workspace/kalman.py) (`build_F`, `build_Q`, `ekf_predict`, `ekf_update`);
[`association.py`](workspace/association.py) (`mahalanobis_distance`, `chi2_gate`,
`associate_and_update`);
[`camera_fusion.py`](workspace/camera_fusion.py) (`is_in_field_of_view`,
`camera_measurement_prediction`, `build_camera_measurement`);
[`track_management.py`](workspace/track_management.py) (`init_track_state_from_meas`,
`update_track_score`, `should_delete_track`). Điều kiện camera không tạo/xoá/đổi score
được phối hợp bởi [`TrackManager`](../platform/fusion_lab/tracking/manager.py).

## Bonus (không bắt buộc)

**Trực quan hoá track/camera update — đề nghị +3 điểm.** Có ba ảnh PNG với frame,
track ID, BEV, vùng phóng to vị trí trước/sau update và ảnh FRONT có chú thích.
Mỗi dòng dưới đây so sánh cùng track ngay trước và ngay sau camera pass của fused,
không so sánh hai tracker độc lập. Score/state và tập ID giữ nguyên qua camera pass.

| Ảnh | Frame / track ID | Lỗi vị trí 3D trước camera (m) | Sau camera (m) |
|---|---|---:|---:|
| [camera_update_026.png](bonus/camera_update_026.png) | 26 / 1 | 0.145158 | 0.055678 |
| [camera_update_131.png](bonus/camera_update_131.png) | 131 / 10 | 0.293761 | 0.109637 |
| [camera_update_150.png](bonus/camera_update_150.png) | 150 / 10 | 0.220523 | 0.014052 |

Ví dụ frame 150: camera update dịch vị trí từ
`(27.583487, 7.178894, 0.690820)` sang `(27.536357, 7.320371, 0.544632)` m,
gần tâm GT `(27.536289, 7.334042, 0.541381)` m hơn. Trên ảnh FRONT, pixel dự đoán
sau update cũng tiến gần observation. Toạ độ và lỗi được lưu trong
[`visualization_evidence.json`](bonus/visualization_evidence.json). Đây là ba ví dụ
được chọn có giảm lỗi, không đại diện cho mọi camera update; kết luận tổng thể dùng
RMSE trên đủ 199 frame.

**Phân tích calibration — đề nghị +4 điểm.** Chạy đủ segment ở 5 mức lệch yaw camera,
giữ cùng detector, seed 0, pixel observations, noise covariance và code E–H. Mỗi
tracker có trạng thái riêng; chỉ thay extrinsic giả định bằng
`T_bad = T_true @ Rz(yaw)`, quay quanh trục up của camera. Lần chạy chấm điểm chuẩn
không có perturbation này.

| Lệch yaw (°) | RMSE 3D (m) | Camera updates thực tế | Mean residual u (px), probe cố định | Median d², probe | Tỷ lệ probe qua FOV + gating |
|---:|---:|---:|---:|---:|---:|
| 0 | 0.135848 | 508 | −12.151 | 1.560 | 100.00% |
| 0.5 | 0.211140 | 320 | −30.788 | 9.213 | 63.19% |
| 2 | 0.176003 | 19 | −86.491 | 76.901 | 2.36% |
| 5 | 0.150262 | 28 | −197.229 | 415.744 | 0.98% |
| 10 | 0.175371 | 46 | −380.496 | 1569.786 | 0.20% |

Trong cả năm full replay: `matches = 502`, `ghosts = 0`, `misses = 239`.
Mỗi mức có **508 cặp probe**: lấy các cặp được camera nominal chấp nhận, đóng băng
`x/P/z` trước update, rồi tính lại residual/Mahalanobis với từng extrinsic. Các probe
không update tracker và không thay đổi pairing, nên minh hoạ bias do phép chiếu
độc lập với sự thay đổi quỹ đạo/gating selection. Đây là nominal association,
không phải GT correspondence độc lập. Mức 10° có 507 cặp trong FOV; các mức còn lại
có 508. `camera_updates` là số update thật của tracker ở từng mức, **không** phải
số probe qua gate.

Residual ngang ngày càng âm cho thấy sai lệch có hệ thống. Gate camera dùng
`χ²(0.995, 2) ≈ 10.5966`: mức 0.5° có median d² gần ngưỡng, còn nhiều update lọt
qua và RMSE tăng rõ. Từ 2° trở lên, median d² lớn hơn nhiều nên phần lớn probe bị
loại; ở 5° RMSE `0.150262 m` gần LiDAR `0.150278 m`, nhất quán với việc camera ít
còn tác dụng. RMSE không tăng đơn điệu theo độ lệch: gate thay đổi lượng update,
và full replay còn thay đổi `x/P` cùng các cặp được gán. Ở 10°, 46 update thật so với
chỉ 1 probe nominal qua gate nhắc rằng gating không chứng minh đúng danh tính cặp
ghép. Không thể kết luận tất cả update còn lại ghép sai chỉ từ thống kê này.

Nominal mean residual không bằng 0 dù calibration chuẩn: tâm hộp 2D và ảnh chiếu
tâm 3D khác nhau, đồng thời track/detector còn sai số. Vì vậy dấu hiệu calibration
là **thay đổi bias so với nominal trên cùng cặp probe**, không yêu cầu residual
nominal chính xác bằng 0.

Bằng chứng: [biểu đồ calibration](bonus/calibration.png),
[bảng tổng hợp CSV](bonus/calibration_summary.csv),
[probe chi tiết](bonus/calibration_probes.csv), `bonus/metrics_yaw_*.json`,
`bonus/records_yaw_*.jsonl`, `bonus/camera_updates_yaw_*.csv`,
[script chạy](bonus/run_experiments.py), [script kiểm tra](bonus/verify_evidence.py),
[hướng dẫn tái lập](bonus/README.md). Bonus nominal tái lập được metrics của runner
gốc; không chỉnh tay artifacts. CVAT chưa có môi trường import/ảnh chụp xác minh,
nên **không đề nghị +3 điểm CVAT**; `tracks_export.json` chỉ là export sơ bộ bằng
helper của platform. Tổng mức bonus đề nghị: **+7/10**, giảng viên xác nhận khi chấm.

## Khai báo sử dụng AI (bắt buộc)

Khai báo theo [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code; OpenAI Codex
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ triển khai Part E–H trong
  `student/workspace/` (EKF predict/update, gating Mahalanobis, phép chiếu camera,
  vòng đời track). Codex hỗ trợ kiểm tra rubric, sửa diễn giải báo cáo, viết script
  thí nghiệm calibration/trực quan hoá trong `student/bonus/`, kiểm tra tính nhất
  quán của artifacts, chạy lại test và hoàn tất Git.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy
  `.venv/Scripts/python.exe -X utf8 -B -m pytest student/tests -q -p no:cacheprovider --disable-warnings`:
  **128 passed**, không còn `xfailed`/`failed`; dùng UTF-8 để fixture tiếng Việt chạy
  đúng trên Windows;
  đối chiếu công thức EKF (`F`, `Q`, `K = P Hᵀ S⁻¹`, `P⁺ = (I − K H) P`), ngưỡng χ² và
  quy tắc cập nhật điểm tồn tại với `fusion_lab/tracking_params.py`; chạy
  `fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0`
  sau thí nghiệm bonus để thu nhận lại số liệu thực nghiệm chuẩn (ghi ở mục
  Tóm tắt kết quả). Chạy `student/bonus/verify_evidence.py` để đối chiếu toàn bộ
  metrics/logs, CSV probe, baseline nominal và toạ độ/lỗi trên ảnh; kiểm tra bằng
  `tools/check_submission.py` sau khi commit.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (không sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push code, báo cáo và bằng chứng lên GitHub
- [ ] Học viên đã nộp link repo + commit hash cuối cùng trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
