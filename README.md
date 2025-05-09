# CILQR方法用于自动驾驶运动规划的原理推导与操作指南
注：为避免引入错误，全文所有矩阵求导运算均采用分子布局
## 1 Vehicle Model
将车辆位置姿态进行建模，其离散化状态空间方程为：

$$
\vec{X}_{k+1}=f(\vec{X_k}, \vec{u_k})=
\begin{pmatrix}
x_k+(v_k \cdot \Delta t+ \frac{1}{2} \dot{v_k} \cdot \Delta t^2) \cdot \cos \theta_k \\
y_k+(v_k \cdot \Delta t+ \frac{1}{2} \dot{v_k} \cdot \Delta t^2) \cdot \sin \theta_k \\
v_k+\dot{v_k} \cdot \Delta t \\
\theta_k + \dot{\theta_k} \cdot \Delta t
\end{pmatrix}
$$

<p align="right">(1)</p>
其中，

$$\vec{X_k}=\begin{pmatrix} x_k \\
y_k \\
v_k \\
\theta_k
\end{pmatrix} $$

<p align="right">(2)</p>

$$\vec{u_k}=\begin{pmatrix} \dot{v_k}\\ 
\dot{\theta_k} \end{pmatrix} $$

<p align="right">(3)</p>
对上述离散系统进行线性化，即在(xk, uk)处进行一阶Taylor展开:

$$
\vec{X}_{k+1} + \delta \vec{X} _ {k+1}=f(\vec{X_k}+ \delta \vec{X_k}, \vec{u_k} +\delta \vec{u_k})=f(\vec{X_k}, \vec{u_k})+\frac{\partial f}{\partial \vec{X_k}} \Big| _{\vec{X_k}} + \frac{\partial f}{\partial \vec{u_k}} \Big| _{\vec{u_k}}
$$

<p align="right">(4)</p>

$$
即 \qquad \delta \vec{X} _{k+1}=\frac{\partial f}{\partial \vec{X_k}} \Big| _{\vec{X_k}}+\frac{\partial f}{\partial \vec{u_k}} \Big| _{\vec{u_k}}
$$

<p align="right">(5)</p>

则车辆模型离散状态空间方程的矩阵A、B分别为：

$$
A_k=\frac{\partial f}{\partial \vec{X_k}}=\begin{pmatrix} \frac{\partial f}{\partial x} & \frac{\partial f}{\partial y} & \frac{\partial f}{\partial v} & \frac{\partial f}{\partial \theta}\end{pmatrix}
=\begin{pmatrix} \frac{\partial f_1}{\partial x} & \frac{\partial f_1}{\partial y} & \frac{\partial f_1}{\partial v} & \frac{\partial f_1}{\partial \theta}\\
\frac{\partial f_2}{\partial x} & \frac{\partial f_2}{\partial y} & \frac{\partial f_2}{\partial v} & \frac{\partial f_2}{\partial \theta}\\
\frac{\partial f_3}{\partial x} & \frac{\partial f_3}{\partial y} & \frac{\partial f_3}{\partial v} & \frac{\partial f_3}{\partial \theta}\\
\frac{\partial f_4}{\partial x} & \frac{\partial f_4}{\partial y} & \frac{\partial f_4}{\partial v} & \frac{\partial f_4}{\partial \theta}
\end{pmatrix}
=\begin{pmatrix}
1 & 0 & \cos \theta_k \cdot \Delta t & -\sin \theta_k \cdot (v_k \cdot \Delta t + \frac{1}{2} v_k \cdot \Delta t^2)\\
0 & 1 & \sin \theta_k \cdot \Delta t & \cos \theta_k \cdot (v_k \cdot \Delta t + \frac{1}{2} v_k \cdot \Delta t^2)\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}
$$

<p align="right">(6)</p>

$$
B_k=\frac{\partial f}{\partial \vec{u_k}}=
\begin{pmatrix}
\frac{\partial f_1}{\partial \dot{v_k}} & \frac{\partial f_1}{\partial \dot{\theta_k}}\\
\frac{\partial f_2}{\partial \dot{v_k}} & \frac{\partial f_2}{\partial \dot{\theta_k}}\\
\frac{\partial f_3}{\partial \dot{v_k}} & \frac{\partial f_3}{\partial \dot{\theta_k}}\\
\frac{\partial f_4}{\partial \dot{v_k}} & \frac{\partial f_4}{\partial \dot{\theta_k}}\\
\end{pmatrix}
=\begin{pmatrix}
\frac{1}{2}\cos \theta_k \cdot \Delta t^2 & 0\\
\frac{1}{2}\sin \theta_k \cdot \Delta t^2 & 0\\
\Delta t & 0\\
0 & \Delta t
\end{pmatrix}
$$

<p align="right">(7)</p>

## 2 Cost Function
### 2.1 原始问题
该运动规划过程原始的最优化问题为

$$
J=\frac{1}{2} (\vec{X_N}-\vec{X^r_N})^T S (\vec{X_N}-\vec{X^r_N})+ \sum_{k=1}^{N-1} \Big[ \frac{1}{2}(\vec{X_k}-\vec{X^r_k})^T Q (\vec{X_k}-\vec{X^r_k}) +\frac{1}{2} \vec{u_k}^T R \vec{u_k} \Big]
$$
$$
s.t \begin{cases}
a_{low} \leq 
[\dot v & \dot \theta]
\begin{bmatrix} 
1 \\ 
0 
\end{bmatrix} 
\leq a_{high} \\
\frac{v \cdot \tan \delta_{min}}{L} \leq
[\dot v & \dot \theta]
\begin{bmatrix} 
1 \\ 
0 
\end{bmatrix}
\leq \frac{v \cdot \tan \delta_{max}}{L}\\
1-\vec{X^O_ {front}}^T P \vec{X^O _ {front}} < 0\\
1-\vec{X^O_ {rear}}^T P \vec{X^O_{rear}} < 0
\end{cases}
$$

<p align="right">(8)</p>

### 2.2 约束转换为惩罚函数
#### 2.2.1 避障约束
(1)避障约束函数推导

避障场景中，将主车近似为两个圆，障碍物考虑其速度，近似为椭圆，如图1所示。

<div align=center> <img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Figures/ego_obs_approximate.jpg" width=800> </div>

<p align="center">图1 避障约束近似示意图</p>


障碍物长轴：

$$
a=\frac{1}{2}l_{obs} + v_{obs} \cdot t_{safe} \cdot \cos \theta_{obs} + s_{safe \_a} + r_{ego}
$$

<p align="right">(9)</p>

障碍物短轴：

$$
b=\frac{1}{2}l_{obs} + v_{obs} \cdot t_{safe} \cdot \sin \theta_{obs} + s_{safe \_b} + r_{ego}
$$

<p align="right">(10)</p>

则避障约束中的矩阵P为

$$
P=
\begin{pmatrix}
\frac{1}{a^2} & 0 & 0 & 0\\
0 & \frac{1}{b^2} & 0 & 0\\
0 & 0 & 0 & 0\\
0 & 0 & 0 & 0
\end{pmatrix}
$$

<p align="right">(11)</p>

主车两个近似圆圆心的位置为：

$$
\vec{X} _ {front}=\begin{pmatrix}
x_{front} \\
y_{front} \\
v_{front} \\
\theta_{front}
\end{pmatrix}
=\begin{pmatrix}
x_{ego} + l_f \cdot \cos \theta_{ego} \\
y_{ego} + l_f \cdot \sin \theta_{ego} \\
v_{ego}
\theta_{ego} \\
\end{pmatrix}
$$

<p align="right">(12)</p>

$$
\vec{X} _ {rear}=\begin{pmatrix}
x_{rear} \\
y_{rear} \\
v_{rear} \\
\theta_{rear}
\end{pmatrix}
=\begin{pmatrix}
x_{ego} - l_r \cdot \cos \theta_{ego} \\
y_{ego} - l_r \cdot \sin \theta_{ego} \\
v_{ego} \\
\theta_{ego}
\end{pmatrix}
$$

<p align="right">(13)</p>

则前后两个近似圆圆心处对主车状态求导如下：

$$
\frac{\partial \vec{X}_{front}}{\partial \vec{X}}=\begin{pmatrix}
\frac{\partial f_1}{\partial \vec{X}} \\
\frac{\partial f_2}{\partial \vec{X}} \\
\frac{\partial f_3}{\partial \vec{X}} \\
\frac{\partial f_4}{\partial \vec{X}}
\end{pmatrix}
=\begin{pmatrix}
\frac{\partial f_1}{\partial \vec{x_1}} & \frac{\partial f_1}{\partial \vec{x_2}} & \frac{\partial f_1}{\partial \vec{x_3}} & \frac{\partial f_1}{\partial \vec{x_4}} \\
\frac{\partial f_2}{\partial \vec{x_1}} & \frac{\partial f_2}{\partial \vec{x_2}} & \frac{\partial f_2}{\partial \vec{x_3}} & \frac{\partial f_2}{\partial \vec{x_4}} \\
\frac{\partial f_3}{\partial \vec{x_1}} & \frac{\partial f_3}{\partial \vec{x_2}} & \frac{\partial f_3}{\partial \vec{x_3}} & \frac{\partial f_3}{\partial \vec{x_4}} \\
\frac{\partial f_4}{\partial \vec{x_1}} & \frac{\partial f_4}{\partial \vec{x_2}} & \frac{\partial f_4}{\partial \vec{x_3}} & \frac{\partial f_4}{\partial \vec{x_4}}
\end{pmatrix}
=\begin{pmatrix}
1 & 0 & 0 & -l_f \sin \theta _{ego} \\
0 & 1 & 0 & l_f \cos \theta _{ego} \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix}
$$

<p align="right">(14)</p>

同理，

$$
\frac{\partial \vec{X}_{rear}}{\partial \vec{X}}
=\begin{pmatrix}
1 & 0 & 0 & -l_r \sin \theta _{ego} \\
0 & 1 & 0 & l_r \cos \theta _{ego} \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix}
$$

<p align="right">(15)</p>

分别将前后近似圆的位置坐标转换到障碍物obstacle坐标系下：

$$
\vec{X}^O_{front}=T \cdot (\vec{X} _ {front} - \vec{X} _ {obs})
=\begin{pmatrix}
\cos \theta _ {obs} & \sin \theta _ {obs} & 0 & 0 \\
-\sin \theta _ {obs} & \cos \theta_{obs} & 0 & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0
\end{pmatrix}
\begin{pmatrix}
x_{front} - x_{obs} \\
y_{front} - y_{obs} \\
0-v_{obs} \\
0-\theta_{obs}
\end{pmatrix}
=\begin{pmatrix}
(x_{front}-x_{obs}) \cdot \cos \theta_{obs} + (y_{front}-y_{obs}) \cdot \sin \theta_{obs} \\
-(x_{front}-x_{obs}) \cdot \sin \theta_{obs} + (y_{front}-y_{obs}) \cdot \cos \theta_{obs} \\
0 \\
0
\end{pmatrix}
$$

<p align="right">(16)</p>

同理，

$$
\vec{X}^O_{rear}=T \cdot (\vec{X}_{rear}-\vec{X} _{obs})
$$

<p align="right">(17)</p>


## 3 Backward Pass
## 4 Forward Pass
## 5 Results
## 6 Commands


