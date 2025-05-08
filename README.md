# CILQR方法用于自动驾驶运动规划的原理推导与操作指南
注：为避免引入错误，全文所有矩阵求导运算均采用分子布局
## 1 Vehicle Model
将车辆位置姿态进行建模，其离散化状态空间方程为：

$$
\vec{X}_{k+1}=f(\vec{X_k}, \vec{u_k})=
\begin{pmatrix}
x_k+(v_k \cdot \Delta t+ \frac{1}{2} \dot{v_k} \cdot \Delta t^2) \cdot \cos(\theta_k) \\
y_k+(v_k \cdot \Delta t+ \frac{1}{2} \dot{v_k} \cdot \Delta t^2) \cdot \sin(\theta_k) \\
v_k+\dot{v_k} \cdot \Delta t \\
\theta_k + \dot{\theta_k} \cdot \Delta t
\end{pmatrix}
$$

<p align="right">(1)</p>
其中，

$$\vec{x_k}=\begin{pmatrix} x_k \\
y_k \\
v_k \\
\theta_k
\end{pmatrix} $$

<p align="right">(2)</p>

$$\vec{u_k}=\begin{pmatrix} \dot{v_k}\\ 
\dot{\theta_k} \end{pmatrix} $$

<p align="right">(3)</p>
对上述离散系统进行线性化，即


## 2 Cost Function
## 3 Backward Pass
## 4 Forward Pass
## 5 Results
## 6 Commands


