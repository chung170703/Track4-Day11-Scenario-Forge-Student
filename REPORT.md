MSSV: 202602985

# Lab 11 · Scenario Forge — báo cáo

Thị trấn của bạn (`meta.json: _run.town`): **B**

---

## 1. Phát lại log có trung thực không? (bước A)

### Bảng số liệu (`evidence/replay.json`)

| Chỉ số | Giá trị | Trích dẫn file: trường |
|---|---|---|
| Số kịch bản log phát lại ($n$) | 400 | `replay.json: n = 400` |
| Kịch bản bỏ vì người lái gây va chạm | 0 | `replay.json: dropped_human_crash_logs = 0` |
| Số kịch bản sử dụng ($n_{used}$) | 400 | `replay.json: n_used = 400` |
| Va chạm khi phát lại (`replay.crashes`) | 110 | `replay.json: replay.crashes = 110` |
| Tiếp xúc phát lại: `rear` | 98 | `replay.json: replay.contact.rear = 98` |
| Tiếp xúc phát lại: `side` | 12 | `replay.json: replay.contact.side = 12` |
| Va chạm trong thế giới phản ứng (`reactive.crashes`) | 0 | `replay.json: reactive.crashes = 0` |
| Tiếp xúc phản ứng | `{}` (không có) | `replay.json: reactive.contact = {}` |
| Va chạm ở cả hai (`crash_in_both`) | 0 | `replay.json: crash_in_both = 0` |

### Giải thích nguyên nhân va chạm giả và kịch bản `replay_only`
- **Vì sao phần lớn va chạm phát lại là `rear` và là va chạm giả:** Trong chế độ phát lại log mở (open-loop replay), quỹ đạo của các phương tiện xung quanh được phát lại cứng theo thời gian đã ghi nhận từ con người. Khi ego (xe tự hành điều khiển bởi planner) phát hiện chướng ngại vật hoặc người đi bộ phía trước, planner phanh chủ động và sớm hơn người lái trong log. Tuy nhiên, xe đi sau (follower/tailgater) trong log phát lại không hề biết xe ego đang phanh; nó tiếp tục lao tới theo vị trí tuyệt đối trong log cũ và húc vào đuôi xe ego (`rear = 98`). Đây là va chạm **giả** (phantom collision) vì ngoài đời thực (và trong mô phỏng phản ứng closed-loop), người lái phía sau có mắt quan sát, thấy xe trước phanh đỏ đèn thì sẽ phanh theo mô hình bám đuôi IDM; minh chứng là trong thế giới phản ứng không hề có va chạm nào xảy ra (`reactive.crashes = 0`).
- **Phân tích một kịch bản trong `replay_only` (Kịch bản #2):**
  - Trích xuất: `replay.json: replay_only[0]` (Scenario 2, `contact = "rear"`, `min_ttc_reactive = 7.4606 s`).
  - Diễn biến: Ở kịch bản này, xe chuyển làn (`cutT0 = 6.26 s`, `cutDur = 3.13 s`, `cutV = 11.59 m/s`) và người đi bộ (`pedT0 = 8.40 s`, `pedV = 1.40 m/s`) xuất hiện phía trước. Planner của ego giảm tốc để nhường đường và giữ an toàn. Trong replay log, xe bám đuôi phía sau (`tgT = 1.30 s`) không giảm tốc theo mà tiếp tục phóng tới theo log con người, đâm vào đuôi ego. Ngược lại, trong thế giới phản ứng (`reactive`), xe bám đuôi kích hoạt giảm tốc IDM theo chuyển động của ego, giữ cự ly an toàn rất lớn với TTC tối thiểu lên tới 7.46 s, hoàn toàn không xảy ra tai nạn.

---

## 2. Kịch bản bất khả thi (bước B)

### Các luật trong `is_feasible` và căn cứ vật lý
1. **Gia tốc ngang xe chuyển làn ($a_{lat} \le \mu g$):**
   - Công thức: $a_{lat} = 2 \cdot |cutDs| / cutDur^2 \le \mu \cdot g$ (với $g = 9.81 m/s^2$).
   - Căn cứ vật lý: Giới hạn vòng tròn ma sát (friction circle) giữa lốp xe và mặt đường theo định luật Coulomb ($F_{lat}^{max} = \mu m g$). Gia tốc chuyển làn ngang không thể vượt quá ma sát mặt đường cho phép; nếu vượt quá, xe sẽ bị trượt lốp, văng đuôi mất kiểm soát.
2. **Gia tốc phanh xe chuyển làn ($cutBrake \le \mu g$):**
   - Căn cứ vật lý: Lực hãm phanh cực đại của xe cũng bị khống chế bởi lực bám ma sát dọc ($a_{brake} \le \mu g$). Xe không thể phanh với gia tốc lớn hơn độ bám của lốp (ví dụ đường trơn $\mu = 0.5$ thì gia tốc phanh tối đa là $4.9 m/s^2$).
3. **Tốc độ đi bộ của người ($pedV \le 2.5 m/s$):**
   - Căn cứ vật lý: Vận tốc đi bộ bình thường của người dao động từ $1.2 - 1.5 m/s$, bước nhanh/chạy vội tối đa khoảng $2.5 m/s$ (~9 km/h). Vượt quá $2.5 m/s$ biến người đi bộ thành vận động viên chạy nước rút, phi lý trong tình huống người đi bộ sang đường thông thường.
4. **Vận tốc xe chuyển làn không âm ($cutV \ge 0$):**
   - Căn cứ vật lý: Xe lưu thông trên làn một chiều chỉ có thể tiến về phía trước, không thể lùi xe trên đường giao thông tốc độ cao.

### Cuộc săn bất khả thi và hệ quả trong CEM
- **Cuộc săn sinh nhiều kịch bản bất khả thi nhất:** `crosswalk_block` với 14 kịch bản vi phạm (`hunt.json: crosswalk_block.infeasible = 14`), so với `bus_bypass` (3), `cut_in` (4), `rear_ended` (8).
- **Vì sao CEM bị kéo về phía đó:** Thuật toán Cross-Entropy Method (CEM) liên tục cập nhật phân phối đề xuất $q$ hướng về các mẫu có điểm số nguy cơ cao nhất (TTC nhỏ nhất, va chạm nghiêm trọng nhất). Các kịch bản phi vật lý (như xe tạt đầu với gia tốc ngang vô lý hoặc người đi bộ phi nhanh tức thời) tạo ra những tình huống hiểm nghèo nhân tạo khiến planner bó tay chịu đâm. Nếu không có bộ lọc `is_feasible`, CEM sẽ coi đây là những mẫu "ưu tú" (elite) và nhanh chóng kéo toàn bộ phân phối đề xuất hội tụ vào vùng phi vật lý đó.
- **Hệ quả nếu đếm chúng là va chạm:** Xác suất tai nạn ước lượng sẽ bị **thổi phồng quá mức** (gross overestimation / false pessimism). Xe tự hành sẽ bị ép phải tối ưu hóa cho những kịch bản không thể xảy ra ngoài đời, dẫn đến hành vi lái xe cực đoan, quá nhút nhát và tê liệt không dám di chuyển (driving paralysis).

---

## 3. Monte Carlo cho bạn biết gì (bước C1)

### Kết quả từ `evidence/mc.json`
- Số lần chạy ($n$): 3000 (`mc.json: n = 3000`).
- Số va chạm ($k$): 4 (`mc.json: crashes = 4`).
- Tỷ lệ ước lượng điểm ($\hat{p}$): $\hat{p} = \frac{4}{3000} \approx 0.001333$ ($1.33 \times 10^{-3}$) (`mc.json: p = 0.0013333333333333333`).
- Khoảng tin cậy Wilson 95%: $[lo, hi] = [0.0005186, 0.0034235]$ (`mc.json: lo = 0.0005186172629797158`, `mc.json: hi = 0.003423534680052167`), tức $[5.19 \times 10^{-4}, 3.42 \times 10^{-3}]$.

### Số lần chạy để khoảng tin cậy hẹp lại còn $\pm 20\%$ của $p$
- Biên độ sai số tương đối $\epsilon = 0.20 \implies E = 0.20 \cdot p = z \cdot \sqrt{\frac{p(1-p)}{n}}$.
- Với mức tin cậy 95% ($z = 1.96$), ta có công thức:
  $$n \ge \frac{z^2 \cdot (1-p)}{\epsilon^2 \cdot p} \approx \frac{1.96^2}{0.04 \cdot p} \approx \frac{96.04}{p}$$
- Thay $\hat{p} \approx 0.001333$:
  $$n \ge \frac{96.04}{0.0013333} \approx 72,030 \text{ lần chạy}$$
  (Cần khoảng **72,000 lần chạy** Monte Carlo).

### Kiểu thất bại chưa xuất hiện và con số đúng cần báo
- Trong 3000 lần chạy, có **4 kiểu thất bại chưa hề xuất hiện** ($k = 0$): `cut_in`, `crosswalk_block`, `rear_ended`, và `other` (`mc.json: per_mode.*.k = 0`). Toàn bộ 4 va chạm xảy ra đều thuộc kiểu `bus_bypass`.
- **Con số đúng cần báo:** Tuyệt đối không được báo bằng $0$ (vì chưa thấy không có nghĩa là xác suất bằng không). Con số chuẩn xác cần báo là **cận trên khoảng tin cậy Wilson 95%** (Rule of Three / Wilson upper bound):
  $$p \le 0.0012789 \approx 1.28 \times 10^{-3} \quad (\text{trích từ } \texttt{mc.json: per\_mode[mode].hi = 0.0012788956648046956})$$

---

## 4. Importance sampling, ESS và độ suy biến (bước C3)

### Bảng số liệu chi tiết từ `evidence/estimate.json`

| Kiểu thất bại | Seed | Xác suất $p$ | Số va chạm ($hits$) | Cỡ mẫu hiệu dụng ($ess$) | Tỷ trọng mẫu lớn nhất ($top\_share$) |
|---|---|---|---|---|---|
| `bus_bypass` | 0 | $7.357 \times 10^{-4}$ | 43 | 145.95 | 0.2219 |
| | 1 | $3.798 \times 10^{-4}$ | 28 | 124.59 | 0.2481 |
| | 2 | $8.974 \times 10^{-4}$ | 49 | 125.42 | 0.2655 |
| `cut_in` | 0 | 0.0 | 0 | 125.21 | 0.0 (cận trên 0.025) |
| | 1 | 0.0 | 0 | 128.57 | 0.0 (cận trên 0.025) |
| | 2 | 0.0 | 0 | 124.05 | 0.0 (cận trên 0.025) |
| `crosswalk_block` | 0 | $1.951 \times 10^{-7}$ | 35 | 124.97 | 0.2655 |
| | 1 | $1.631 \times 10^{-6}$ | 25 | 114.79 | **0.6674** |
| | 2 | $9.153 \times 10^{-8}$ | 19 | 114.18 | 0.4944 |
| `rear_ended` | 0 | 0.0 | 0 | 112.03 | 0.0 (cận trên 0.025) |
| | 1 | 0.0 | 0 | 117.09 | 0.0 (cận trên 0.025) |
| | 2 | 0.0 | 0 | 121.32 | 0.0 (cận trên 0.025) |

Tổng xác suất qua 3 seed:
- Seed 0: $p_{total} = 7.359 \times 10^{-4}$ ($[lo, hi] = [3.041 \times 10^{-4}, 2.447 \times 10^{-3}]$) (`estimate.json: seeds[0].total`)
- Seed 1: $p_{total} = 3.814 \times 10^{-4}$ ($[lo, hi] = [1.285 \times 10^{-4}, 1.913 \times 10^{-3}]$) (`estimate.json: seeds[1].total`)
- Seed 2: $p_{total} = 8.975 \times 10^{-4}$ ($[lo, hi] = [2.673 \times 10^{-4}, 2.807 \times 10^{-3}]$) (`estimate.json: seeds[2].total`)
- Trung vị qua 3 seed: **$7.359 \times 10^{-4}$**.

### Đánh giá độ đồng thuận và hiện tượng `top_share`
- **Độ đồng thuận giữa 3 seed và với Monte Carlo:**
  - Cả 3 seed đồng thuận chặt chẽ về bậc độ lớn cho `bus_bypass` ($\sim 3.8 \times 10^{-4} - 9.0 \times 10^{-4}$) và hoàn toàn nằm gọn trong khoảng tin cậy 95% của Monte Carlo ($[5.19 \times 10^{-4}, 3.42 \times 10^{-3}]$, $\hat{p}_{MC} = 1.33 \times 10^{-3}$).
  - Với `crosswalk_block`, cả 3 seed cùng xác nhận đây là kiểu thất bại cực hiếm ($\sim 10^{-7} - 10^{-6}$), điều mà Monte Carlo 3000 mẫu hoàn toàn mù tịt ($k = 0$).
- **Hiện tượng `top_share` > 0.5:**
  - Xảy ra ở **Seed 1 của `crosswalk_block`** với $top\_share = 0.6674$ (66.74%).
  - *Nguyên nhân:* Trọng số mẫu trong IS là $w_i = p(x_i)/q(x_i)$. Sau 8 vòng lặp CEM, phân phối đề xuất $q$ bị co hẹp mạnh về vùng va chạm. Khi một mẫu va chạm rơi vào vị trí có xác suất tự nhiên $p(x)$ tương đối cao nhưng mật độ đề xuất $q(x)$ tại đó lại rất thấp (ở đuôi của $q$), tỷ số trọng số $w_i$ vọt lên khổng lồ và chiếm tới hơn 66.7% tổng trọng số của cả 600 mẫu đánh giá. Khi đó, ước lượng kỳ vọng bị chi phối hoàn toàn bởi một mẫu cá biệt duy nhất (suy biến trọng số).
- **So sánh ngân sách (`budget`):**
  - Ngân sách IS: `estimate.json: budget = 16800` lần chạy (gồm tìm kiếm CEM: $4 \times 8 \times 300 = 9600$, và đánh giá IS: $4 \times 3 \times 600 = 7200$).
  - Để Monte Carlo thông thường đạt cùng độ chính xác cho kiểu cực hiếm `crosswalk_block` ($p \approx 2 \times 10^{-7}$), số lần chạy cần thiết là $n \approx 96 / (2 \times 10^{-7}) \approx 480,000,000$ lần chạy! IS giúp tiết kiệm ngân sách tính toán gấp gần **30,000 lần**.
- **Phần `other` trong tổng:**
  - Lấy từ `mc.json` (`estimate.json: other.source = "mc.json"`, $k = 0, n = 3000, p = 0.0, hi = 0.0012789$).
  - Không săn `other` bằng CEM vì `other` là một tập hợp hỗn tạp các va chạm ngẫu nhiên không có cấu trúc hình học hay tham số hóa cụ thể. CEM không thể định hướng tối ưu hóa một phân phối đề xuất $q$ cho một mục tiêu không có quy luật không gian.

---

## 5. Các kiểu thất bại (bước C2 + 3D)

### 1. Kiểu `bus_bypass` (Lách xe buýt đâm người đi bộ)

![Bus Bypass Crash](file:///Users/chung140204/Track4-Day11-Scenario-Forge-Student/evidence/shots/ch9-bus-bypass.jpg)

- **Chuỗi sự kiện:** Xe buýt đỗ tại trạm che khuất tầm nhìn sang lề đường; xe ego lách sang trái vượt qua sườn xe buýt ở tốc độ khoảng 7.5 m/s thì người đi bộ bất ngờ bước ra từ ngay trước mũi xe buýt; khoảng cách quá gần kết hợp góc mù khiến ego phát hiện muộn và đâm trực diện vào người đi bộ tại $t = 12.4$ s (Kịch bản B-01).

### 2. Kiểu `crosswalk_block` (Kẹt trên vạch sang đường đâm người đi bộ)

![Crosswalk Block Crash](file:///Users/chung140204/Track4-Day11-Scenario-Forge-Student/evidence/shots/ch9-crosswalk-block.jpg)

- **Chuỗi sự kiện:** Người đi bộ đang qua đường trên vạch sơn; xe ego tiến tới vạch bộ hành nhưng luồng giao thông phía trước bị ách tắc hoặc dừng đột ngột khiến ego không thể hoàn tất việc băng qua nút giao, dẫn tới việc xe đè lên vạch người đi bộ và va chạm với người đi bộ đang đi dở lộ trình tại $t = 20.95$ s (Kịch bản C-01).

### Kiểu không tìm thấy và hiện tượng sụp đổ kiểu (Mode Collapse)
- **Kiểu cuộc săn không tìm thấy:**
  - `cut_in`: Cuộc săn `cut_in` tìm thấy 55 va chạm nhưng **tất cả 100% đều bị gán nhãn là `bus_bypass`** (`hunt.json: cut_in.found = {"bus_bypass": 55}`). Không có va chạm `cut_in` thực sự.
  - `rear_ended`: Cuộc săn `rear_ended` tìm thấy 8 va chạm `bus_bypass` và 2 `crosswalk_block`, tìm thấy 0 `rear_ended` (`hunt.json: rear_ended.found = {"bus_bypass": 8, "crosswalk_block": 2}`).
- **Mức độ tin cậy:** Rất cao đối với `rear_ended` vì ego trong thế giới phản ứng phanh mượt mà và xe sau có IDM giữ khoảng cách nên hầu như không thể bị đâm đuôi trừ khi vi phạm vật lý. Với `cut_in`, do xe tạt đầu bị ràng buộc vật lý ($a_{lat} \le \mu g$), planner có đủ thời gian phản ứng nhường đường, hoặc kịch bản bị lu mờ do va chạm xe buýt phía trước xảy ra sớm hơn.
- **Nếu chỉ chạy một cuộc săn chung cho mọi kiểu:** Thuật toán CEM sẽ bị **sụp đổ kiểu (mode collapse)** hoàn toàn vào `bus_bypass` (vốn có xác suất cao nhất $\sim 10^{-3}$ và dễ kích hoạt nhất). Kiểu cực hiếm nhưng nghiêm trọng là `crosswalk_block` ($p \sim 10^{-7}$) sẽ bị áp đảo tuyệt đối và không bao giờ được phát hiện. Do đó, bắt buộc phải phân rã và săn riêng biệt từng kiểu thất bại.

---

## 6. Sửa planner (bước E + 3D)

### Bảng so sánh Policy gốc và Policy được chọn (`evidence/fix.json`)

| Chỉ số | Policy gốc (`baseline_test`) | Policy được chọn (`chosen_test` / `{"cutinLat": 3.0}`) | Ghi chú & Nguồn |
|---|---|---|---|
| **Tham số chính** | `cutinLat = 2.15` | `cutinLat = 3.0` | Mở rộng biên nhận diện xe nhập làn |
| **TRAIN: xác suất thất bại $fail$** | $4.915 \times 10^{-4}$ | $8.371 \times 10^{-8}$ | `fix.json: baseline_train / table[7]` |
| **TRAIN: cận trên $fail\_hi$** | $8.217 \times 10^{-4}$ | $1.667 \times 10^{-7}$ | Giảm hơn 4900 lần trên TRAIN |
| **TEST: xác suất thất bại $fail$** | $1.424 \times 10^{-3}$ | $8.312 \times 10^{-7}$ | Giảm hơn 1700 lần trên TEST! |
| **TEST: $fail\_hi\_seen$ (đã thấy)** | $2.338 \times 10^{-3}$ | $1.503 \times 10^{-6}$ | Cận trên các kiểu đã quan sát |
| **TEST: $fail\_hi$ (chuẩn kiểm định)**| $3.617 \times 10^{-3}$ | **$0.03878$** | Phạt do có kiểu `unseen` |
| **Quãng đường di chuyển ($progress$)**| 212.85 m | 212.87 m | Duy trì đầy đủ vận hành (+0.02 m) |
| **Số khung hình phanh gấp ($harsh$)** | 8.74 frames | 8.14 frames | Êm ái hơn (-0.60 frames) |

### Vì sao $fail\_hi$ của bản sửa lại lớn hơn $fail$ của policy gốc trên TEST?
- Trên tập TEST 400 mẫu, bản sửa `cutinLat = 3.0` hoạt động quá xuất sắc đối với kiểu `bus_bypass`: **không có bất kỳ va chạm nào xảy ra** (`hits = 0`).
- Theo nguyên tắc cận bảo thủ thống kê cho biến cố chưa quan sát (`unseen`), kiểu `bus_bypass` bị phạt một cận trên Rule of Three rất lớn:
  $$upper\_zero\_hits = \frac{3}{n_{eff}} = \frac{3}{0.2 \times 400} = 0.0375 \quad (\texttt{fix.json: chosen\_test.bus\_bypass.upper\_zero\_hits})$$
- Khi cộng dồn vào tổng kiểm định:
  $$fail\_hi = fail\_hi\_seen + 0.0375 = 1.503 \times 10^{-6} + 0.0375 \approx 0.03878$$
- Con số 0.03878 này là **hình phạt mẫu hữu hạn (finite sample penalty)** khi cố gắng chứng minh một biến cố có tỷ lệ bằng 0 trên cỡ mẫu $N=400$, chứ không phản ánh năng lực thực tế của xe. Thực chất, xác suất thất bại điểm của xe đã giảm hơn 1700 lần (từ $1.42 \times 10^{-3}$ xuống $8.31 \times 10^{-7}$) và cận trên các kiểu đã thấy `fail_hi_seen` giảm hơn 1500 lần!

### Ảnh 3D bóng mờ (Ghost Comparison)

![Ghost Fix Comparison](file:///Users/chung140204/Track4-Day11-Scenario-Forge-Student/evidence/shots/ch9-fix-ghost.jpg)

- **Minh họa:** Kịch bản B-01 tại $t = 12.4$ s. Bản sửa (xe thật màu xanh) nhận diện chướng ngại vật từ xa và dừng lại an toàn trước người đi bộ, trong khi policy gốc (bóng mờ màu xám) phản ứng muộn và lao thẳng vào người đi bộ.

### Núm chỉnh "nghe thì an toàn" nhưng phản tác dụng
- **Núm:** `cautionV = 5.0` (giảm tốc độ qua khu vực xe buýt từ $7.5 m/s$ xuống $5.0 m/s$) (`fix.json: table[0]`).
- **Hậu quả:**
  - Tỷ lệ thất bại `fail` tăng gần gấp đôi từ $4.915 \times 10^{-4}$ lên **$8.587 \times 10^{-4}$**!
  - Số khung hình phanh gấp `harsh` tăng vọt từ 7.62 lên **13.22 frames**.
  - Quãng đường di chuyển `progress` sụt giảm nghiêm trọng từ 218.84 m xuống chỉ còn **172.86 m**.
- **Giải thích bằng cách thế giới phản ứng:**
  Khi hạ `cautionV` xuống 5.0 m/s, xe ego bò quá chậm qua cạnh xe buýt, làm kéo dài thời gian phơi nhiễm (exposure duration) của xe trong vùng điểm mù của người đi bộ. Xe đi chậm khiến người đi bộ từ đầu xe buýt bước ra đâm đúng vào hông hoặc đầu xe ego, trong khi ở tốc độ bình thường ego đã dứt điểm vượt qua vị trí đó từ trước. Đồng thời, việc rà phanh bò chậm bất ngờ kích động các xe bám đuôi phía sau phản ứng phanh gấp liên tục.

---

## 7. Từ mô phỏng ra đường thật

### Ba giả định ảnh hưởng nhiều nhất trong `space.py`

1. **Tỷ lệ người đi bộ quan sát đường trước khi bước qua đầu xe buýt (`pedLook`):**
   - *Dữ liệu thật thay thế & Nguồn:* Dữ liệu quan sát hành vi tự nhiên của người đi bộ tại trạm xe buýt từ camera giao thông hoặc tập dữ liệu mở JAAD (Joint Attention for Autonomous Driving) và PIE (Pedestrian Intention Estimation).
   - *Hướng sai lệch:* Trong mô phỏng, tỷ lệ người nhìn đường chỉ khoảng 20–45% ($p_{bin}$ thấp), phần lớn cắm mặt đi thẳng. Ngoài đời thực, đa số người đi bộ đều ngó đầu qua sườn xe buýt để quan sát trước khi bước tiếp (`pedLook = 1`). Do đó, con số rủi ro của mô phỏng đang **bị thổi phồng cao hơn thực tế** (quá bi quan / pessimistic).
2. **Hệ số ma sát mặt đường ($\mu \in \{0.5, 0.8\}$):**
   - *Dữ liệu thật thay thế & Nguồn:* Dữ liệu đo ma sát bánh xe từ cảm biến phanh ABS/ESP của các đội xe thương phẩm (fleet telematics), kết hợp dữ liệu khí tượng bề mặt đường (mưa, dầu mỡ, băng giá).
   - *Hướng sai lệch:* Mô phỏng giả định đường lý tưởng $\mu = 0.8$ hoặc ướt đều $\mu = 0.5$. Ngoài đời, vũng dầu, nắp cống ướt hoặc lá cây mục có thể làm $\mu < 0.3$. Nếu mô phỏng giả định $\mu$ quá cao, quãng đường phanh tính toán sẽ ngắn hơn thực tế, khiến con số rủi ro của mô phỏng **bị đánh giá thấp hơn thực tế** (quá lạc quan / optimistic).
3. **Phân phối khoảng cách nhận diện xe chuyển làn (`cutGap`, `tgT`):**
   - *Dữ liệu thật thay thế & Nguồn:* Tập dữ liệu quỹ đạo giao thông tự nhiên quy mô lớn như highD, exiD hoặc Waymo Open Dataset.
   - *Hướng sai lệch:* Ngoài đời, tài xế chỉ tạt đầu khi nhìn thấy khoảng trống đủ an toàn (gap acceptance động). Mô phỏng lấy mẫu tham số tạt đầu có thể sinh ra các tình huống ép làn cực đoan hơn tự nhiên, làm con số xác suất va chạm bị **thiên lệch cao hơn thực tế**.

---

## 8. Giới hạn

### Ba điều mà xác suất báo không nói
1. **Động lực học xe bị đơn giản hóa (Kinematic point-mass model):** Mô phỏng coi xe là chất điểm động học, phản hồi phanh tức thì, không có độ trễ thủy lực của bàn đạp phanh (actuator lag 150–250 ms), không xét trượt ngang phi tuyến của lốp (tire slip) hay hiện tượng chúi đầu xe (pitch) làm thay đổi tải trọng trục bánh khi phanh gấp.
2. **Hệ thống cảm biến hoàn hảo (Bounding-box sensing không nhiễu):** Xe tự hành trong mô phỏng nhận diện hoàn hảo vị trí và vận tốc vật thể ngay khi có đường ngắm (line-of-sight). Ngoài đời thực, cụm camera/LiDAR phải đối mặt với nhiễu ảnh, trễ xử lý của mạng nơ-ron (perception tracking latency 100–200 ms), và hiện tượng nhấp nháy phát hiện (detection flickering).
3. **Mô hình hành vi thiếu tính tương tác xã hội (No social negotiation):** Các xe khác và người đi bộ chỉ di chuyển theo luật IDM hoặc vận tốc cố định, không có giao tiếp phi ngôn ngữ (ánh mắt, cử chỉ tay, nháy đèn pha) để thương lượng quyền ưu tiên qua đường như tài xế con người.

### Nếu được thêm một tuần, bạn sẽ đo gì tiếp?
- Đo lường **tác động của độ trễ nhận thức và phản hồi cơ cấu chấp hành (Perception & Actuation Latency Sensitivity):**
  - Cài đặt mô hình trễ ngẫu nhiên: độ trễ nhận diện nhận thức $t_{perc} \in [50, 200]$ ms và độ trễ phản hồi phanh thủy lực $t_{act} \in [100, 250]$ ms.
  - Chạy lại toàn bộ pipeline để định lượng mức tăng theo cấp số nhân của xác suất tai nạn $p$ theo từng mili-giây trễ, từ đó thiết lập ngân sách thời gian thực (real-time deadline) bắt buộc cho phần cứng máy tính trên xe tự hành.

---

## 9. Rắc rối của bạn (bước G + chương 10)

### 1. Mô tả rắc rối và căn cứ
- **Rắc rối dựng:** Xe đạp di chuyển ở mép trong làn E2 ($d = -0.2$ m, vận tốc $v = 4.2$ m/s), bất ngờ lạng sang trái xâm lấn làn E1 ($swerve \in [1.4, 2.3]$ m) trong khoảng thời gian $swDur = 2.0$ s khi xe ego tiến tới cự ly kích hoạt $trig \in [10, 20]$ m trên đoạn đường $x \in [-10, 35]$ m.
- **Tình huống ngoài đời:** Xe đạp hoặc xe máy đi sát mép đường bất ngờ đánh lái gấp sang làn ô tô để tránh ổ gà, nắp cống gồ ghề, hoặc chướng ngại vật ven đường trong đô thị.
- **Căn cứ vì sao planner xử lý kém:** Planner IDM chỉ giám sát chướng ngại vật dọc theo làn E1 và chỉ bắt đầu coi xe làn bên là nguy cơ khi chúng đã vượt qua ngưỡng khoảng cách ngang `cutinLat = 2.15` m. Với cự ly kích hoạt gần ($10-14$ m) và tốc độ ego đang chạy $10-12.5$ m/s, thời gian phản ứng TTC chỉ còn dưới 1 giây. Việc thiếu logic dự đoán ý định (intention prediction) khiến planner phanh quá trễ.

### 2. Giả định không gian và tần suất (`hazard.json`)
- `space`:
  - `h0_x`: $[-10.0, 35.0]$ m (đoạn đường thẳng tốc độ ổn định trước trạm xe buýt).
  - `h0_swerve`: $[1.4, 2.3]$ m (lạng đủ sâu vào làn E1 nhưng gia tốc ngang $a_{lat} = 2 \cdot swerve / swDur^2 \le 2 \times 2.3 / 4 = 1.15 m/s^2 < 3.0 m/s^2$, đảm bảo xe đạp không trượt ngã).
  - `h0_trig`: $[10.0, 20.0]$ m.
- `exposure`: **$0.04$** (0.04 lần mỗi chuyến đi đô thị, trích dẫn báo cáo an toàn xe đạp của CPSC/NHTSA và phân tích quỹ đạo tự nhiên của Johnson et al., 2014; Dozza et al., 2016 về tần suất xe đạp né vật cản ven đường khoảng 3–5 lần trên 100 chuyến đi).

### 3. Kết quả khai thác kịch bản (`evidence/mine.json`)
- Tỷ lệ kịch bản khả thi: **`accepted = 0.994`** ($99.4\% \ge 50\%$, 1491/1500 kịch bản khả thi) (`mine.json: accepted = 0.994`, `infeasible = 9`).
- **So sánh Policy gốc (`baseline`) và Policy được chọn (`chosen` / `cutinLat: 3.0`):**

| Chỉ số | Policy gốc (`baseline`) | Policy được chọn (`chosen`) | Thay đổi |
|---|---|---|---|
| Tổng số lần chạy khả thi ($n$) | 1491 | 1491 | — |
| Tổng số va chạm ($crashes$) | **268** | **51** | **Giảm 81.0%** |
| Va chạm tránh được ($avoidable$) | 266 | 49 | Giảm 81.6% |
| Va chạm bất khả kháng ($unavoidable$) | 2 | 0 | Giảm về 0 |
| Va chạm không quy kết ($unattributed$) | 0 | 2 | — |
| Va chạm tốc độ thấp ($low\_speed$) | 0 | 1 | — |
| **$P(\text{lỗi planner} \mid \text{rắc rối})$** | **$0.1784$** ($17.84\%$) | **$0.0329$** ($3.29\%$) | **Giảm 5.4 lần** |
| Khoảng tin cậy Wilson 95% | $[0.1598, 0.1987]$ | $[0.0249, 0.0432]$ | Tách biệt hoàn toàn |
| Tần suất mỗi chuyến ($per\_trip$) | $0.00714$ ($7.14 \times 10^{-3}$) | $0.00131$ ($1.31 \times 10^{-3}$) | Giảm mạnh |
| Cận trên mỗi chuyến ($per\_trip\_hi$) | $0.00795$ ($7.95 \times 10^{-3}$) | $0.00173$ ($1.73 \times 10^{-3}$) | — |

- **So sánh với tần suất lách xe buýt:**
  - Ở policy gốc, tần suất mỗi chuyến của rắc rối xe đạp là $7.14 \times 10^{-3}$, cao gấp **5 lần** so với tần suất tai nạn lách xe buýt ($1.42 \times 10^{-3}$ trong `fix.json: baseline_test.bus_bypass.p`).
  - Khi áp dụng policy sửa (`cutinLat: 3.0`), tần suất rắc rối xe đạp giảm xuống chỉ còn $1.31 \times 10^{-3}$, đưa mức độ an toàn tương đương với tình huống xe buýt.

### 4. Vì sao "tránh được" ($avoidable$) là chuẩn lạc quan?
- **Khái niệm:**
  - `avoidable`: Va chạm mà một tài xế mẫu mực (oracle) có thể tránh được nếu biết trước thời điểm nguy cơ phát sinh (`fire_t`) và phanh tối đa ($a = -\mu g$).
  - `unavoidable`: Va chạm bất khả kháng do vi phạm khoảng cách vật lý tối thiểu (phanh tối đa ngay từ đầu vẫn đâm).
  - `unattributed`: Va chạm thứ cấp với các đối tượng khác.
- **Vì sao là chuẩn lạc quan (nói đủ cả hai hướng):**
  1. *Đếm nhiều hơn (quá lạc quan về khả năng của planner thật):* Người lái mẫu mực có thông tin "toàn tri", biết trước tích tắc rắc rối bắt đầu và đạp phanh khẩn cấp với gia tốc cực đại ngay lập tức. Ngoài đời, một planner thật không thể đạp phanh giật cục liên tục mỗi khi thấy xe đạp, dẫn đến việc chuẩn này coi mọi pha xử lý không phanh kịp của planner là "lỗi", làm **thổi phồng số va chạm tránh được**.
  2. *Đếm ít đi (quá bi quan về phương án xử lý):* Bài test oracle chỉ thử nghiệm duy nhất hành vi **phanh dọc**, hoàn toàn không xét phương án **đánh lái tránh né** (lateral swerve). Ngoài đời, khi làn đường còn rộng, con người hoặc planner thông minh có thể lách nhẹ sang trái để vượt qua an toàn mà không cần dừng xe. Việc cấm đánh lái khiến một số kịch bản bị phán quyết sai thành "không thể tránh".

### 5. Hình ảnh lúc va chạm (Chương 10)

![Custom Hazard Bike Swerve Crash](file:///Users/chung140204/Track4-Day11-Scenario-Forge-Student/evidence/shots/ch10-custom-hazard.jpg)

- **Minh họa:** Kịch bản GB-01 tại thời điểm $t = 7.0$ s. Xe đạp H0 lạng từ mép làn E2 sang làn E1 của xe ego; ego không kịp dừng và xảy ra va chạm.

### 6. Trải nghiệm tự lái và Đề xuất cách sửa
- **Nếu tự lái con người:** Người lái sẽ chủ động quan sát xe đạp từ cự ly > 30 m, rà nhẹ phanh giảm tốc xuống ~8 m/s, đồng thời chủ động đánh lái dịch thân xe sang bên trái tim đường (lateral offset 0.5–1.0 m) để tạo khoảng cách an toàn trước khi vượt, hoàn toàn tránh được va chạm.
- **Cách sửa bằng núm chỉnh:** Núm `cutinLat = 3.0` (trong policy được chọn) đã giảm được 81% số vụ va chạm vì mở rộng tầm quét ngang, giúp phát hiện xe đạp sớm hơn.
- **Logic mới cần thêm cho planner:**
  1. **Tạo khoảng đệm an toàn ngang (Lateral Clearance Cushion):** Bổ sung quy tắc khi phát hiện đối tượng dễ bị tổn thương (VRU - xe đạp/người đi bộ) ở làn liền kề, planner tự động dịch chuyển tim quỹ đạo tham chiếu sang trái $0.5$ m nếu làn bên trái thông thoáng.
  2. **Dự đoán ý định lạng làn (Swerve Intention Prediction):** Thêm bộ ước lượng vận tốc ngang $\dot{y}$ cho xe đạp; ngay khi phát hiện vector vận tốc ngang hướng vào làn xe ego vượt quá ngưỡng $0.3 m/s$, kích hoạt phanh phòng ngừa (defensive deceleration) ngay lập tức mà không cần đợi xe đạp cắt qua ranh giới phân làn.
