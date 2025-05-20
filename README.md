# CILQR方法用于自动驾驶运动规划的原理推导与操作指南
注：为避免引入错误，全文所有矩阵求导运算均采用分子布局
## 0 Results

项目成果展示如下：

<div align=center> <img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Results/Following.gif" width=500><img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Results/MultiObs.gif" width=500> </div>

<div align=center> <img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Results/LaneChange.gif" width=500><img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Results/Overtake.gif" width=500> </div>

<p align="center">项目成果展示</p>



本项目代码在跟车、多障碍物躲避、变道、超车这些常见场景下均表现良好，满足使用需求。由于项目使用C++ 在ROS架构下实现，所以使用rviz做成果展示，其中绿色矩形为主车，灰色矩形为npc车辆，尺寸均按照tesla model3设置，主车前面的绿色曲线为实时规划路径，可以看出项目在静态、动态场景下均有良好表现，可以根据决策结果不同设置不同参数，达到不同的速度与路径选择。

表1所示为本项目代码在各个测试场景下的平均迭代次数和平均求解时间，可以看出各场景下的**平均迭代次数均小于10**， 其中多障碍物躲避和超车场景下的平均迭代次数均小于5。表1所示的平均求解时间为处理器i7-9750h下的运行速度，单位为毫秒。结合平均迭代次数来看，算法可以快速收敛至最优解，但实际求解时间仍高于预期值，表明当前性能瓶颈可能源于硬件限制或处理器算力未充分调用，后续会将此类优化作为重点改进方向。

**表1 迭代次数与求解时间**

|场景 |平均迭代次数 |平均求解时间/ms (i7-9750H)
|:------:|:------:|:------:|
|跟车|7|418|
|多障碍物躲避|4.5|529.5|
|变道|5|320|
|超车|2|215.5|

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

### 2.2 约束转换为障碍函数
#### 2.2.1 避障约束
**(1)避障约束函数推导**

避障场景中，将主车近似为两个圆，障碍物考虑其速度，近似为椭圆，如图1所示。

<div align=center> <img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Figures/ego_obs_approx.jpg" width=800> </div>

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

将式(8)中的第三四个不等式约束对状态量求导

$$
\begin{cases}
  C_f(\vec{X}) =1 - \vec{X^O_{front}}^T \cdot P \cdot \vec{X^O} _ {front} <0  \\
  C_r(\vec{X}) =1 - \vec{X^O _ {rear}}^T \cdot P \cdot \vec{X^O}_{rear} <0
\end{cases}
$$

<p align="right">(18)</p>

则

$$
\frac{\partial C_f}{\partial \vec{X}}=\frac{\partial \vec{C_f}}{\partial \vec{X}^O_{front}} \cdot \frac{\partial \vec{X}^O_{front}}{\partial \vec{X}_{front}} \cdot \frac{\partial \vec{X} _{front}}{\partial \vec{X}} = -2 \vec{X^O _{front}}^T \cdot P \cdot T \cdot \frac{\partial \vec{X} _{front}}{\partial \vec{X}}
$$

<p align="right">(19)</p>

$$
\frac{\partial C_r}{\partial \vec{X}}=\frac{\partial \vec{C_r}}{\partial \vec{X}^O_{rear}} \cdot \frac{\partial \vec{X}^O_{rear}}{\partial \vec{X}_{rear}} \cdot \frac{\partial \vec{X} _{rear}}{\partial \vec{X}} = -2 \vec{X^O _{rear}}^T \cdot P \cdot T \cdot \frac{\partial \vec{X} _{rear}}{\partial \vec{X}}
$$

<p align="right">(20)</p>

将式(11)(14)(16)代入式(19), 式(11)(15)(17)代入式(20)，即可求出避障约束对状态向量的一阶导。

**(2) 避障约束函数线性化**
在Xk处对上述 $C_f,C_r$ 函数进行泰勒一阶展开，将约束函数线性化：

$$
C_f(\vec{X}_k + \delta \vec{X}_k) = C_f(\vec{X}_k) + \frac{\partial C_f}{\partial \vec{X}} \Big| _{\vec{X}=\vec{X}_k} \cdot \delta \vec{X}_k
$$

<p align="right">(21)</p>

$$
\delta C_f(\vec{X}_k)=\frac{\partial C_f}{\partial \vec{X}} \Big| _{\vec{X}_k} \cdot \delta \vec{X}_k
$$

<p align="right">(22)</p>

同理，

$$
\delta C_r(\vec{X}_k)=\frac{\partial C_r}{\partial \vec{X}} \Big| _{\vec{X}_k} \cdot \delta \vec{X}_k
$$

<p align="right">(23)</p>

将线性化后的避障约束函数写为

$$
\begin{cases}
f_{Cf}(\vec{X})=\frac{\partial C_f}{\partial \vec{X}} \Big| _ {\vec{X}_ k} \cdot \vec{X} \\
f_{Cr}(\vec{X})=\frac{\partial C_r}{\partial \vec{X}} \Big| _{\vec{X}_k} \cdot \vec{X}
\end{cases}
$$

<p align="right">(24)</p>

**(3) 避障约束转换为barrier function**

为了将约束问题转化为无约束问题，将式(8)(24)中的避障约束不等式转换为障碍函数(barrier function)添加到目标函数中，前后两个近似圆对应的障碍函数分别为

$$
\begin{cases}
b_f(\vec{X})=q_1 e^{q_2 f_{Cf}(\vec{X})} \\
b_r(\vec{X})=q_1 e^{q_2 f_{Cr}(\vec{X})}
\end{cases}
$$

<p align="right">(25)</p>

上述barrier function对状态向量的一阶导为

$$
\begin{cases}
\frac{\partial b_f(\vec{X})}{\partial \vec{X}}=q_1 q_2e^{q_2 f_{Cf}(\vec{X})} \cdot \frac{\partial f_{Cf}(\vec{X})}{\partial \vec{X}}= q_1 q_2 e^{q_2 f_{Cf}(\vec{X})} \cdot \frac{\partial C_f}{\partial \vec{X}} \Big| _ {\vec{X}_ k} \\
\frac{\partial b_r(\vec{X})}{\partial \vec{X}}=q_1 q_2e^{q_2 f_{Cr}(\vec{X})} \cdot \frac{\partial f_{Cr}(\vec{X})}{\partial \vec{X}}= q_1 q_2 e^{q_2 f_{Cr}(\vec{X})} \cdot \frac{\partial C_r}{\partial \vec{X}} \Big| _{\vec{X}_k}
\end{cases}
$$

<p align="right">(26)</p>

将式(18)(19)(20)(24)代入式(26)可求出避障约束barrier function对状态向量的一阶导。

由于已经对避障约束函数进行了线性化，所以避障函数 $C_f,C_r$ 对状态向量的二阶导为0，即

$$
\begin{cases}
\frac{\partial ^2 {C_f}}{\partial \vec{X}^2} = 0 \\
\frac{\partial ^2 {C_r}}{\partial \vec{X}^2} = 0
\end{cases}
$$

<p align="right">(27)</p>

所以避障约束barrier function对状态向量的二阶导为

$$
\begin{cases}
\frac{\partial ^2 {b_f}(\vec{X})}{\partial \vec{X}^2} = q_1 q_2^2 e ^{q_2 f_{Cf}(\vec{X})} \cdot \big( \frac{\partial C_f}{\partial \vec{X}} \big|_ {\vec{X}_ k} \big)^T \cdot \big( \frac{\partial C_f}{\partial \vec{X}} \big|_ {\vec{X}_ k} \big) \\
\frac{\partial ^2 {b_r}(\vec{X})}{\partial \vec{X}^2} = q_1 q_2^2 e ^{q_2 f_{Cr}(\vec{X})} \cdot \big( \frac{\partial C_r}{\partial \vec{X}} \big|_ {\vec{X}_ k} \big)^T \cdot \big( \frac{\partial C_r}{\partial \vec{X}} \big|_{\vec{X}_k} \big)
\end{cases}
$$

<p align="right">(28)</p>

将式(18)(19)(20)(24)代入式(28)中，即可求出barrier function的二阶导。

上述所求的barrier function的一二阶导会在使用iLQR方法进行迭代求解时用到。

### 2.2 控制量约束
本节的主要目的是将式(8)中的第一二个不等式约束转换为障碍函数形式作为目标函数的一部分，将约束问题转换为无约束问题。

由于控制约束本身就是线性的，因此无需进行线性化近似，直接转换为指数形式的障碍函数(barrier function)。首先将控制约束拆解为以下四个不等式：

$$
\begin{cases}
C_1(\vec{u})=\dot{v} - a_{high} \leq 0 \\
C_2(\vec{u}) = a_{low} -\dot{v} \leq 0 \\
C_3(\vec{u}) = \dot \theta - \frac{v \cdot \tan \delta _{max}}{L} \leq 0 \\
C_4(\vec{u}) =\frac{v \cdot \tan \delta _{min}}{L} -\dot \theta \leq 0
\end{cases}
$$

<p align="right">(29)</p>

再将上述四个控制约束函数转换为障碍函数：

$$
\begin{cases}
b_1(\vec{u})= q_1 e^{q_2 C_1(\vec{u})} \\
b_2(\vec{u})= q_1 e^{q_2 C_2(\vec{u})} \\
b_3(\vec{u})= q_1 e^{q_2 C_3(\vec{u})} \\
b_4(\vec{u})= q_1 e^{q_2 C_4(\vec{u})} \\
\end{cases}
$$

<p align="right">(30)</p>

控制约束障碍函数的一阶导为

$$
\begin{cases}
\frac{\partial b_1(\vec{u})}{\partial \vec{u}} = q_1 q_2 e^{q_2 C_1(\vec{u})} \cdot \frac{\partial C_1{\vec{u}}}{\partial \vec{u}} \\
\frac{\partial b_2(\vec{u})}{\partial \vec{u}} = q_1 q_2 e^{q_2 C_2(\vec{u})} \cdot \frac{\partial C_2{\vec{u}}}{\partial \vec{u}} \\
\frac{\partial b_3(\vec{u})}{\partial \vec{u}} = q_1 q_2 e^{q_2 C_3(\vec{u})} \cdot \frac{\partial C_3{\vec{u}}}{\partial \vec{u}} \\
\frac{\partial b_4(\vec{u})}{\partial \vec{u}} = q_1 q_2 e^{q_2 C_4(\vec{u})} \cdot \frac{\partial C_4{\vec{u}}}{\partial \vec{u}}
\end{cases}
$$

<p align="right">(31)</p>

其中，

$$
\begin{cases}
\frac{\partial C_1(\vec{u})}{\partial \vec{u}} = \begin{pmatrix} \frac{\partial C_1(\vec{u})}{\partial u_1} & \frac{\partial C_1(\vec{u})}{\partial u_2} \end{pmatrix} = \begin{pmatrix} 1 & 0 \end{pmatrix} \\
\frac{\partial C_2(\vec{u})}{\partial \vec{u}} = \begin{pmatrix} \frac{\partial C_2(\vec{u})}{\partial u_1} & \frac{\partial C_2(\vec{u})}{\partial u_2} \end{pmatrix} =\begin{pmatrix} -1 & 0 \end{pmatrix} \\
\frac{\partial C_3(\vec{u})}{\partial \vec{u}} = \begin{pmatrix} \frac{\partial C_3(\vec{u})}{\partial u_1} & \frac{\partial C_3(\vec{u})}{\partial u_2} \end{pmatrix} =\begin{pmatrix} 0 & 1 \end{pmatrix} \\
\frac{\partial C_4(\vec{u})}{\partial \vec{u}} = \begin{pmatrix} \frac{\partial C_4(\vec{u})}{\partial u_1} & \frac{\partial C_4(\vec{u})}{\partial u_2} \end{pmatrix} =\begin{pmatrix} 0 & -1 \end{pmatrix} \\
\end{cases}
$$

<p align="right">(32)</p>

将式(29)(32)代入式(31), 求出控制约束障碍函数的一阶导。

控制约束障碍函数的二阶导为

$$
\begin{cases}
\frac{\partial ^2 b_1(\vec{u})}{\partial \vec{u}^2} = q_1 q_2^2 e ^{q_2 C_1(\vec{u})} \cdot \Big(\frac{\partial C_1(\vec{u})}{\partial \vec{u}} \Big)^T \cdot \Big( \frac{\partial C_1(\vec{u})}{\partial \vec{u}} \Big) \\
\frac{\partial ^2 b_2(\vec{u})}{\partial \vec{u}^2} = q_1 q_2^2 e ^{q_2 C_2(\vec{u})} \cdot \Big(\frac{\partial C_2(\vec{u})}{\partial \vec{u}} \Big)^T \cdot \Big( \frac{\partial C_2(\vec{u})}{\partial \vec{u}} \Big) \\
\frac{\partial ^2 b_3(\vec{u})}{\partial \vec{u}^2} = q_1 q_2^2 e ^{q_2 C_3(\vec{u})} \cdot \Big(\frac{\partial C_3(\vec{u})}{\partial \vec{u}} \Big)^T \cdot \Big( \frac{\partial C_3(\vec{u})}{\partial \vec{u}} \Big) \\
\frac{\partial ^2 b_4(\vec{u})}{\partial \vec{u}^2} = q_1 q_2^2 e ^{q_2 C_4(\vec{u})} \cdot \Big(\frac{\partial C_4(\vec{u})}{\partial \vec{u}} \Big)^T \cdot \Big( \frac{\partial C_4(\vec{u})}{\partial \vec{u}} \Big)
\end{cases}
$$

<p align="right">(33)</p>

将式(29)(32)代入式(33)可得到控制约束障碍函数的二阶导，在后续使用iLQR方法迭代求导时会用到。

### 2.3 最终问题

经过上述处理，该运动规划问题由2.1节的约束优化问题转换为式(34)所示的无约束优化问题。

$$
J=\frac{1}{2} \Big( \vec{X}_ N - \vec{X}^r _ N \Big) ^T S \Big(\vec{X}_ N - \vec{X} ^ r_N \Big) + \sum_{k=1}^{N-1} \Big[ \frac{1}{2} {\Big( \vec{X}_ k - \vec{X} _ k ^ r)} ^ T Q ( {\vec{X}_ k - \vec{X}^r_k} \Big) + \frac{1}{2} \vec{u}_ k^T R \vec{u} _ k + \sum_{m=0} ^{M} \Big( b _{fm} (\vec{X}_k) + b _{rm} (\vec{X}_k) \Big) + b_1(\vec{u}_k) + b_2(\vec{u}_k) + b_3(\vec{u}_k) + b_4(\vec{u}_k)
 \Big]
$$

<p align="right">(34)</p>

其中M为该时刻主车周围障碍物总数， $b_{fm}$ , $b_{rm}$ 分别为主车两个近似圆的避障障碍函数， $b_1, b_2, b_3, b_4$ 分别为控制约束障碍函数。目标函数对状态向量和控制向量的一阶导、二阶导求法均已在2.2节给出。以下给出使用iLQR方法求解问题的过程。

## 3 Backward Pass

### 3.1 代价函数处理

根据LQR先验知识，代价函数J由末端代价和过程代价两部分组成，根据式(8)(34), 可将代价函数写为

$$
J=\ell _f(\vec{X}_N) + \sum _{k=1}^{N-1} \ell (\vec{X}_k, \vec{u}_k)
$$

<p align="right">(35)</p>

### 3.2 Q函数与贝尔曼方程

when $k=N$, cost to go function 

$$ V_N= \ell _f(\vec{X}_N) = \frac{1}{2} \Big( \vec{X}_N - \vec{X}_N^r \Big)^T S \Big( \vec{X}_N - \vec{X}_N^r \Big) $$

<p align="right">(36)</p>

when $k \neq N$, cost to go function

$$
V_k = min \\{ \ell (\vec{X}_ k, \vec{u}_ k) + V_{k+1} \big( f(\vec{X}_k, \vec{u}_k) \big) \\}
$$

<p align="right">(37)</p>

令

$$
Q_k(\vec{X}_k, \vec{u}_k) = \ell (\vec{X}_k, \vec{u} _ k) + V _{k+1} \big[ f(\vec{X}_k, \vec{u}_k) \big]
$$

<p align="right">(38)</p>

在 $(\vec{X}_k, \vec{u}_k)$ 处进行二阶泰勒展开：

$$
Q(\vec{X}_k + \delta \vec{X}_k, \vec{u}_k + \delta \vec{u}_k) 
= Q(\vec{X}_k, \vec{u}_k) + \frac{\partial Q}{\partial \vec{X}} \Big| _ {(\vec{X}_k, \vec{u}_k)} \cdot \delta \vec{X}_k+ 
\frac{\partial Q}{\partial \vec{u}} \Big| _ {(\vec{X}_k, \vec{u}_k)} \cdot \delta \vec{u}_k +
\frac{1}{2} {(\delta \vec{X}_k)}^T \cdot \frac{\partial ^2 Q}{\partial \vec{X}^2} \bigg| _ {(\vec{X}_k, \vec{u}_k)} (\delta \vec{X}_k) +
\frac{1}{2} {(\delta \vec{u}_k)}^T \cdot \frac{\partial ^2 Q}{\partial \vec{u}^2} \bigg| _ {(\vec{X}_k, \vec{u}_k)} (\delta \vec{u}_k) + 
\frac{1}{2} {(\delta \vec{X}_k)}^T \cdot \frac{\partial ^2 Q}{\partial \vec{u} \partial \vec{X}} \bigg| _ {(\vec{X}_k, \vec{u}_k)} (\delta \vec{u}_k) +
\frac{1}{2} {(\delta \vec{u}_k)}^T \cdot \frac{\partial ^2 Q}{\partial \vec{X} \partial \vec{u}} \bigg| _ {(\vec{X}_k, \vec{u}_k)} (\delta \vec{X}_k)
$$

<p align="right">(39)</p>

则

$$
\begin{aligned}
\delta Q(\vec{X}_k, \vec{u}_k) &= Q_x \cdot \delta \vec{X}_k +Q_u \cdot \delta \vec{u}_k + \frac{1}{2}(\delta \vec{X}_k)^T Q _{xx} (\delta \vec{X}_k) +
\frac{1}{2}(\delta \vec{u}_k)^T Q _{uu} (\delta \vec{u}_k) + \frac{1}{2}(\delta \vec{X}_k)^T Q _{ux} (\delta \vec{u}_k) +
\frac{1}{2}(\delta \vec{u}_k)^T Q _{xu} (\delta \vec{X}_k) \\\
&=\begin{bmatrix} Q_x & Q_u \end{bmatrix} \begin{bmatrix} \delta \vec{X}_k \\\ \delta \vec{u}_k \end{bmatrix} + 
\frac{1}{2} \begin{bmatrix} \delta \vec{X}_k \\\ \delta \vec{u}_k \end{bmatrix}^T \begin{bmatrix} Q _{xx} & Q _{ux} \\\ Q _{xu} & Q _{uu}\end{bmatrix}
\begin{bmatrix} \delta \vec{X}_k \\\ \delta \vec{u}_k \end{bmatrix} &
\end{aligned}
$$

<p align="right">(40)</p>

其中

$$
Q_X = \frac{\partial \ell}{\partial \vec{X}} \Big| _ {\vec{X}_k, \vec{u}_k} + \frac{\partial V _{k+1}}{\partial \vec{X} _{k+1}} \cdot \frac{\partial \vec{X} _{k+1}}{\partial \vec{X} _{k}}
$$

<p align="right">(41)</p>

$$
Q_u = \frac{\partial \ell}{\partial \vec{u}} \Big| _ {\vec{X}_k, \vec{u}_k} + \frac{\partial V _{k+1}}{\partial \vec{X} _{k+1}} \cdot \frac{\partial \vec{X} _{k+1}}{\partial \vec{u} _{k}}
$$

<p align="right">(42)</p>

$$
Q_{XX} = \frac{\partial Q_X}{\partial \vec{X}} = \frac{\partial ^2 \ell}{\partial \vec{X}^2} \Big| _{\vec{X}_k, \vec{u}_k} + \frac{\partial ^2 V _{k+1}}{\partial \vec{X} _{k+1} \partial \vec{X}_k} \cdot \frac{\partial \vec{X} _{k+1}}{\partial \vec{X} _{k}}
=\frac{\partial ^2 \ell}{\partial \vec{X}^2} \Bigg| _{\vec{X}_k, \vec{u}_k} + \Big( \frac{\partial \vec{X} _{k+1}}{\partial \vec{X}_k} \Big)^T \cdot \frac{\partial ^2 V _{k+1}}{\partial \vec{X} _{k+1} ^2} \cdot \big( \frac{\partial \vec{X} _{k+1}}{\partial \vec{X}_k} \big)
$$

<p align="right">(43)</p>

$$
Q_{uu} = \frac{\partial Q_u}{\partial \vec{u}} = \frac{\partial ^2 \ell}{\partial \vec{u}^2} \Big| _{\vec{X}_k, \vec{u}_k} + \frac{\partial ^2 V _{k+1}}{\partial \vec{X} _{k+1} \partial \vec{u}_k} \cdot \frac{\partial \vec{X} _{k+1}}{\partial \vec{u} _{k}}
=\frac{\partial ^2 \ell}{\partial \vec{u}^2} \Bigg| _{\vec{X}_k, \vec{u}_k} + \Big( \frac{\partial \vec{X} _{k+1}}{\partial \vec{u}_k} \Big)^T \cdot \frac{\partial ^2 V _{k+1}}{\partial \vec{X} _{k+1} ^2} \cdot \big( \frac{\partial \vec{X} _{k+1}}{\partial \vec{u}_k} \big)
$$

<p align="right">(44)</p>

$$
Q_{Xu} = \frac{\partial Q_X}{\partial \vec{u}} = \frac{\partial ^2 \ell}{\partial \vec{X} \partial \vec{u}} \Big| _{\vec{X}_k, \vec{u}_k} + \frac{\partial ^2 V _{k+1}}{\partial \vec{X} _{k+1} \partial \vec{u}_k} \cdot \frac{\partial \vec{X} _{k+1}}{\partial \vec{X} _{k}}
=\frac{\partial ^2 \ell}{\partial \vec{X} \partial \vec{u}} \Bigg| _{\vec{X}_k, \vec{u}_k} + \Big( \frac{\partial \vec{X} _{k+1}}{\partial \vec{u}_k} \Big)^T \cdot \frac{\partial ^2 V _{k+1}}{\partial \vec{X} _{k+1} ^2} \cdot \big( \frac{\partial \vec{X} _{k+1}}{\partial \vec{X}_k} \big)
$$

<p align="right">(45)</p>

$$
Q_{uX}=Q_{Xu}^T
$$

<p align="right">(46)</p>

其中， $\quad \frac{\partial \vec{X} _{k+1}}{\partial \vec{X} _{k}} =A_k \quad $, 即式(6);  $\quad \frac{\partial \vec{X} _{k+1}}{\partial \vec{u} _{k}} =B_k \quad $, 即式(7)。

要求使得 $\delta Q$ 最小的 $\delta \vec{u}$ ，即是要求令 $\frac{\partial (\delta Q)}{\partial (\delta \vec{u})}=0$ 的最优控制量 $\delta \vec{u}^*$ 。

$$
\frac{\partial (\delta Q)}{\partial (\delta \vec{u})} = Q_u+(\delta \vec{u})^T Q _{uu} + (\delta \vec{X})^T Q _{uX} = 0
$$

<p align="right">(47)</p>

求出

$$
\delta \vec{u}_ k^* = -(Q_{uu}^{-1})^T \big[ Q_u^T + Q_{Xu} \cdot (\delta \vec{X}_k) \big]
$$

<p align="right">(48)</p>

令

$$
\begin{cases}
K=-(Q_{uu}^{-1})^T \cdot Q_{Xu} \\
d= -(Q_{uu}^{-1})^T \cdot Q_u^T
\end{cases}
$$

<p align="right">(49)</p>

则 $\delta \vec{u}_k^*$ 可写为

$$
\delta \vec{u}_k^* = K \cdot \delta \vec{X}_k +d
$$
<p align="right">(50)</p>

由于 $V_k = min \\{ Q_k(\vec{X}_k, \vec{u}_k) \\}$ , 所以 $\delta V_k = min \\{ \delta Q_k(\vec{X}_k, \vec{u}_k) \\}$ ,将式(48)代入式(40)求出 $\delta Q _ {min}$ , 即 $\delta V$ :

$$
\begin{aligned}
\delta V &= min \\{ \delta Q_k(\vec{X} _ k, \vec{u} _ k) \\} \\
&= Q_X \cdot \delta \vec{X} _ k + Q _ u \cdot \delta \vec{u} _ k ^ * + \frac{1}{2} (\delta \vec{X} _ k)^T Q_{XX} (\delta \vec{X} _ k) + \frac{1}{2} (\delta \vec{u} _ k ^ * )^T Q_{uu} (\delta \vec{u} _ k ^ * ) + \frac{1}{2} (\delta \vec{X} _ k)^T Q_{uX} (\delta \vec{u} _ k ^ *) + \frac{1}{2} (\delta \vec{u} _ k ^ *)^T Q_{Xu} (\delta \vec{X}_ k) \\ 
&= Q_X \cdot \delta \vec{X}_ k + Q_u(K \cdot \delta \vec{X}_ k +d) + \frac{1}{2} (\delta \vec{X} _ k)^T Q_{XX} (\delta \vec{X} _ k) + \frac{1}{2} (K \cdot \delta \vec{X}_ k +d) ^ T Q_{uu} (K \cdot \delta \vec{X}_ k +d) + \frac{1}{2} (\delta \vec{X} _ k) ^ T \cdot Q_{uX} (K \cdot \delta \vec{X}_ k +d) + \frac{1}{2}(K \cdot \delta \vec{X}_ k +d) ^ T Q_{Xu} (\delta \vec{X}_k) \\
&= \Big( Q_X + Q _u K + d^T Q _{uu} K + d ^ T Q _ {Xu} \Big) \delta \vec{X} _ k + \frac{1}{2}(\delta \vec{X} _ k) ^ T \big[ Q _ {XX} + K ^ T Q _ {uu} K + Q _ {uX} K + K ^T Q _ {Xu} \big] (\delta \vec{X}_k) + Q_u d + \frac{1}{2} d^T Q _{uu} d
\end{aligned}
$$

<p align="right">(51)</p>

由此推导出式(41)~式(46)中的 $\frac{\partial V_{k+1}}{\partial \vec{X}_{k+1}}$ , $\quad \frac{\partial ^ 2 V _ {k+1}}{\partial \vec{X} _ {k+1} ^ 2}$ 的求法如下：

①当 $k=N$ 时， $\frac{\partial V_{k+1}}{\partial \vec{X}_{k+1}}, \quad \frac{\partial ^ 2 V _ {k+1}}{\partial \vec{X} _ {k+1} ^ 2}=0$ 

②当 $k=N-1$ 时， 

$$
\frac{\partial V_{k+1}}{\partial \vec{X}_{k+1}} = \frac{\partial \ell _f (\vec{X} _ N)}{\partial \vec{X} _ N} = \big( \vec{X} _ N - \vec{X} _ N ^ r \big) ^ T S
$$

<p align="right">(52)</p>

$$
\frac{\partial ^ 2 V _ {k+1}}{\partial \vec{X} _ {k+1} ^ 2} = \frac{\partial ^ 2 \ell _f (\vec{X} _ N)}{\partial \vec{X} _ N ^ 2} =S 
$$

<p align="right">(53)</p>

③当 $k \leq N-2$ 时，说明此时 $V_{N-1}$已经算完。

在 $(\vec{X}_k, \vec{u}_k)$ 处对 $V_k$ 进行二阶泰勒展开：

$$
V_k ( \vec{X}_k + \delta \vec{X}_k, \quad \vec{u}_k + \delta \vec{u}_k ) = V_k ( \vec{X}_k, \vec{u} _ k ) + \frac{\partial V}{\partial \vec{X}} \Bigg| _ {(\vec{X}_k, \vec{u}_k)} \cdot \delta \vec{X}_k + \frac{1}{2} (\delta \vec{X}_k) ^ T \cdot \frac{\partial ^ 2 V}{\partial \vec{X} ^ 2} \Bigg| _ {(\vec{X}_k, \vec{u}_k)} \cdot (\delta \vec{X}_k)
$$

<p align="right">(54)</p>

$$
\delta V_k(\vec{X}_k, \vec{u} _ k) = \frac{\partial V}{\partial \vec{X}} \Bigg| _ {(\vec{X}_k, \vec{u}_k)} \cdot \delta \vec{X}_k + \frac{1}{2} (\delta \vec{X}_k) ^ T \cdot \frac{\partial ^ 2 V}{\partial \vec{X} ^ 2} \Bigg| _ {(\vec{X}_k, \vec{u}_k)} \cdot (\delta \vec{X}_k)
$$

<p align="right">(55)</p>

结合式(51)与式(55), 可得：

$$
\frac{\partial V}{\partial \vec{X}} \Bigg| _ {(\vec{X}_k, \vec{u}_k)} =  Q_X + Q _u K + d^T Q _{uu} K + d ^ T Q _ {Xu}
$$

<p align="right">(56)</p>

$$
\frac{\partial ^ 2 V}{\partial \vec{X} ^ 2} \Bigg| _ {(\vec{X}_k, \vec{u}_k)} = Q _ {XX} + K ^ T Q _ {uu} K + Q _ {uX} K + K ^T Q _ {Xu}
$$

<p align="right">(57)</p>

$$
\Delta V = \frac{1}{2} d^T Q _{uu} d + Q_u d
$$

<p align="right">(58)</p>

$\Delta V$ 为余项。

### 3.3 Backward Pass算法流程总结

<div align=center> <img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Figures/backward_persudo.jpg" width=600> </div>

## 4 Forward Pass

Backward Pass过程计算出了 $0$ ~ $N-1$ 时刻的最优控制量 $\delta \vec{u} _ k ^ *$ ， Forward Pass阶段需要根据最优控制量序列、车辆模型及参考轨迹序列计算出一条目标轨迹。过程如下：

令 $\delta \vec{X}_k= \vec{X}_k ^ {new} - \vec{X} ^ r _ k$

<p align="right">(59)</p>

其中， $\vec{X}_k ^ {new}$ 为本次迭代计算出的轨迹点， $\vec{X} ^ r _ k$ 为参考轨迹点。

则根据第3部分的计算，

$$
\delta \vec{u}_k ^ {new} = K \cdot \delta \vec{X}_k + \alpha \cdot d, \qquad \alpha \in [0,1]
$$

<p align="right">(60)</p>

$$
\vec{u}_k ^ {new} = \vec{u}_k + \delta \vec{u}_k
$$

<p align="right">(61)</p>

$$
\vec{X}_{k+1} ^ {new} = f(\vec{X}_k ^ {new}, \vec{u}_k ^ {new})
$$

<p align="right">(62)</p>

其中， $f(\vec{\overline X}_k, \vec{\overline u}_k)$ 参考式(1)。由此计算出一系列标称轨迹点。

式(60)中的 $\alpha$ 为线搜索的迭代步长，用于在Backward Pass获得的解附近搜索最优解，当未达到终止条件时，每次 $\alpha$ 的更新规则为

$$
\alpha = \gamma \cdot \alpha, \quad usually \quad \gamma = 0.5
$$

<p align="right">(63)</p>

当满足终止条件时，更新目标轨迹为新计算出的轨迹 $\vec{X}_k ^ {new}$ 、控制输入为新计算出的 $\vec{u}_k ^ {new}$ ,并同步更新路径代价 $J$ 。

Forward Pass算法流程总结如下：

<div align=center> <img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Figures/forward_pass_persudo.jpg" width=600> </div>

## 5 Project Architecture

为更好地贴近实车环境，本项目采用C++编程，将仿真场景与运动规划分为两个进程，使用ROS架构实现进程间的通信。同时为了配合控制模块，每帧都进行了规划起点的确定和轨迹拼接、规划与控制采用了不同的计算频率。项目整体架构如下：

<div align=center> <img src="https://github.com/saxijing/cilqr/blob/main/data/display_materials/Figures/project_architecture.jpg" width=800> </div>

<p align="center">图2 项目架构</p>

## 6 Commands

四个测试场景的运行命令如下：

**(1)Following:**

roslaunch launch following_scene.launch

roslaunch launch cilqr_planner_following.launch

**(2)Obstacles Avoidance:**

roslaunch launch multi_obstacles_scene.launch

roslaunch launch cilqr_planner_multi_obstacles.launch

**(3)Lane Change:**

roslaunch launch lane_change_scene.launch

roslaunch launch cilqr_planner_lane_change.launch

**(4)Overtake:**

roslaunch launch overtake_scene.launch

roslaunch launch cilqr_planner_overtake.launch
