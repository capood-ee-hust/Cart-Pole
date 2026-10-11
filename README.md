 # Thiết kế bộ điều khiển con lắc ngược trên xe (Cart-Pole) 

> MATLAB · Simulink · Simscape Multibody · Control System Toolbox

Thiết kế, cài đặt và so sánh ba bộ điều khiển giữ con lắc ngược trên xe ở vị trí thẳng đứng:

- LQR
- PID 
- Linear MPC 


---

## 1. Mô hình toán

### 1.1 Các tham số

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| $M$ | 0.5 kg | khối lượng xe |
| $m$ | 0.5 kg | khối lượng con lắc |
| $l$ | 0.3 m | khoảng cách từ khớp tới khối tâm con lắc |
| $g$ | 9.81 m/s² | gia tốc trọng trường |
| $k_J$ | 1/12 | hệ số quán tính quanh khối tâm: $J_c = k_J\, m l^2$ |
| $J$ | 0.04875 kg·m² | quán tính quanh khớp: $J = (1 + k_J)\, m l^2$ |
| $F_{max}$ | 30 N | giới hạn lực, chung cho mọi bộ ĐK |
| $T_s$ | 0.01 s | chu kỳ lấy mẫu của bộ ĐK |

Tham số $k_J$ mô tả hình dạng con lắc: $k_J = 0$ là quả nặng trên thanh không khối lượng, $k_J = 1/3$ là thanh đồng chất có khớp ở một đầu. 

### 1.2 Phương trình phi tuyến

Phương trình Lagrange viết dưới dạng ma trận khối lượng:

$$
\begin{bmatrix} M+m & -ml\cos q \\ -ml\cos q & J \end{bmatrix}
\begin{bmatrix} \ddot x \\ \ddot q \end{bmatrix} =
\begin{bmatrix} F - ml\,\dot q^2 \sin q \\ mgl\sin q \end{bmatrix}
$$

### 1.3  Mô hình tuyến tính quanh vị trí thẳng đứng

Quanh điểm $X = 0$, $F = 0$ (với $\sin q \approx q$, $\cos q \approx 1$, $\dot q^2 \approx 0$), đặt $D_0 = (M+m)J - (ml)^2$, ta có:

$$
\dot X = A X + B F,\qquad
A = \begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & \dfrac{m^2 g l^2}{D_0} & 0 & 0 \\
0 & \dfrac{(M+m)\,mgl}{D_0} & 0 & 0
\end{bmatrix},\qquad
B = \begin{bmatrix} 0 \\ 0 \\ J/D_0 \\ ml/D_0 \end{bmatrix}
$$

Thay số với tham số mặc định:

$$
A = \begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & 8.41 & 0 & 0 \\
0 & 56.06 & 0 & 0
\end{bmatrix},\qquad
B = \begin{bmatrix} 0 \\ 0 \\ 1.857 \\ 5.714 \end{bmatrix}
$$

Hai hàm truyền từ lực tới vị trí xe và tới góc con lắc:

$$
\frac{X(s)}{F(s)} = \frac{J s^2 - mgl}{D_0\, s^2 \left(s^2 - \lambda^2\right)},\qquad
\frac{Q(s)}{F(s)} = \frac{ml}{D_0 \left(s^2 - \lambda^2\right)},\qquad
\lambda^2 = \frac{(M+m)\,mgl}{D_0}
$$



---

---

## 2. Thiết kế bộ điều khiển 

### 2.1 LQR

Bộ LQR tìm gain $K$ tối thiểu hóa hàm chi phí trên mô hình tuyến tính, với $e = X - X_{ref}$ và $X_{ref} = [x_{ref}, 0, 0, 0]^T$:

$$
J_{LQR} = \int_0^\infty \left( e^T Q e + R F^2 \right) dt,
\qquad
F = \mathrm{sat}_{F_{max}}\!\left(-K e\right)
$$




### 2.2 PID 

$$
F = \mathrm{sat}_{F_{max}}\Big(
\underbrace{-\left(K_{p,q}\, q + K_{d,q}\, \dot q\right)}_{\text{vòng góc (PD)}}
\;+\;
\underbrace{K_{p,x}\, e_x + K_{i,x} \textstyle\int e_x\, dt + K_{d,x}\, \dot x}_{\text{vòng vị trí (PID)}}
\Big),\qquad e_x = x - x_{ref}
$$

Các lựa chọn thiết kế:

- **Vòng góc** nhanh, giữ con lắc thẳng đứng. **Vòng vị trí** chậm hơn, đưa xe về điểm đặt.
- **Chống bão hòa tích phân kiểu back-calculation:** $\dot I = K_{i,x} e_x + K_{aw}\,(F - F_{unsat})$.



### 2.3 MPC tuyến tính

Ở mỗi chu kỳ $T_s$, bộ MPC giải bài toán tối ưu trên tầm dự báo $N$ bước, mỗi bước dài $dt$:

$$
\min_{F_0,\dots,F_{N-1}} \; \sum_{k=0}^{N-1} dt\left(e_k^T Q e_k + R F_k^2\right) + e_N^T P_f\, e_N
\quad\text{s.t.}\quad
e_{k+1} = A_d e_k + B_d F_k,\quad |F_k| \le F_{max}
$$

- $A_d$, $B_d$ là mô hình tuyến tính rời rạc hóa ZOH với bước $dt$.
- $P_f$ là chi phí cuối, lấy từ nghiệm Riccati của LQR rời rạc. Nó xấp xỉ phần chi phí sau tầm dự báo.
- Chỉ lực đầu tiên $F_0$ được áp dụng, rồi bài toán được giải lại ở chu kỳ sau.


---

## 3. Kết quả mô phỏng 

Trong file result với $q_0 = 35$ với các đồ thị tương ứng


## 4. Hướng dẫn chạy 



Chạy một bộ ĐK:

```matlab
startUp                          
out = sim('Cart_Pole');
plotResults(out, 'Names', {'LQR'}) 
```

So sánh cả ba bộ: trong `cartPoleConfig.m`, mục 7, đặt `P.exp.controllers = {'LQR', 'PID', 'MPC'};` (tắt Fast Restart trước khi chạy), rồi gõ:

```matlab
collectData                      % mỗi lần chạy lưu một file trong results/
compareControllers               % đồ thị so sánh, bảng chỉ tiêu, xuất .png và .csv
```

---

## 5. Tài liệu tham khảo

Dự án xây dựng trên mã nguồn mở **Cart_Pole_Simulink**. Model Simulink/Simscape (`Cart_Pole.slx`) và các file hình học `.STL` từ repo https://github.com/junhengl/Cart_Pole_Simulink

