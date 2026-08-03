---
layout: default
title: 第 3 章 旋转运动与 SO(3)
---

# 第 3 章 旋转运动与 SO(3)

> 课程来源：EECS C106A 第 2 周 ｜ 关键词：旋转矩阵、SO(3) 群、四元数、欧拉角、指数映射、罗德里格斯公式 ｜ 预计阅读时长：~45 分钟

## 1. 导论

机器人末端执行器在工作空间中的姿态、IMU 输出的姿态角、视觉里程计估计的位姿……所有这些问题的共同核心只有一个：**如何用数学精确地描述"一个刚体相对于另一个刚体转了多少"**。第 2 章我们建立了线性代数工具——向量、矩阵、线性映射；从本章开始，这些工具开始直接刻画真实的运动。

本章聚焦**纯旋转运动**：刚体绕某一点（通常取质心或原点）转动时，所有点之间的距离保持不变，物体的"姿态"可以用一个 3×3 旋转矩阵来描述。我们将依次学习：旋转矩阵与三维旋转群 SO(3)（第 2.1 节）、由旋转向量经指数映射得到旋转矩阵的"指数坐标"与罗德里格斯公式（第 2.2 节），以及欧拉角、四元数等其他常用表示（第 2.3 节）。MLS 教材第 2 章 1-3 节正是本章的直接对应内容。

这一章之所以重要，是因为它是理解后续一切的基础：第 4 章把旋转与平移合成为 SE(3) 齐次变换；第 5 章的正运动学、第 6 章的雅可比、第 7 章的动力学，全部建立在这套旋转表示之上。更实际地说，几乎所有机器人软件库（ROS 的 tf、Eigen 几何模块、各种位姿估计器）都围绕"旋转如何表示、如何复合、如何与矩阵互转"这三件事展开。把本章各表示之间的转换练熟，后面读代码、写控制器都会顺畅得多。

在动手之前，先问一个看似抽象却很实际的问题：为什么需要那么多表示？答案藏在工程取舍里。旋转矩阵最规范、便于代数推导与误差分析，但用 9 个数表达 3 个自由度，冗余大，且长期数值运算后各列会慢慢"不正交"，必须周期性重新正交化；欧拉角最直观、适合人机交互与参数化搜索，但存在万向节锁，且插值不平滑；轴角/指数坐标只用 3 个数，在旋量（twist）与李群方法中不可替代，但对大角度不够直观；四元数只有 4 个数、无奇异、复合快、便于插值，代价是必须时刻归一化。机器人学里一种常见工程实践是：**存储与运算用四元数或旋转矩阵，显示与输入用欧拉角，推导与优化用指数坐标**。

课程来源方面，EECS C106A（UC Berkeley 的《Introduction to Robotics》）在第 2 周引入旋转矩阵、欧拉角、四元数与齐次变换，作为第 1 周刚体运动学课程的延续。本章覆盖的正是该周前半段的内容。

## 2. 关键概念与记号

### 2.1 旋转矩阵

设 A 为惯性（固定）坐标系，B 为固连在刚体上的体坐标系。把 B 的三个主轴在 A 中的坐标向量 $$x_{ab}, y_{ab}, z_{ab} \in \mathbb{R}^3$$ 按列拼成一个 3×3 矩阵：

$$R_{ab} = \begin{bmatrix} x_{ab} & y_{ab} & z_{ab} \end{bmatrix}$$

这个矩阵就叫**旋转矩阵 (rotation matrix)**。由于坐标系的三个轴彼此正交、且为单位向量（右手系），旋转矩阵满足两条关键性质：

$$R^\top R = RR^\top = I, \qquad \det R = +1$$

第一条是正交性：各列两两正交、范数为 1；第二条利用行列式等于混合积这一事实，右手系保证 $$\det R = +1$$ 而非 $$-1$$。

### 2.2 SO(3) 群

所有满足上述两性质的 3×3 实矩阵构成集合：

$$SO(3) = \{R \in \mathbb{R}^{3\times 3} : R^\top R = I, \det R = +1\}$$

SO 是 special orthogonal（特殊正交）的缩写，"特殊"指行列式为 +1 而非 ±1。SO(3) 在矩阵乘法下构成一个**群 (group)**——四条群公理逐一成立：

1. **封闭性**：$$R_1, R_2 \in SO(3) \Rightarrow R_1 R_2 \in SO(3)$$；
2. **单位元**：单位阵 $$I$$；
3. **逆元**：$$R^{-1} = R^\top \in SO(3)$$；
4. **结合律**：矩阵乘法天然满足。

因此 SO(3) 是嵌入在 $$\mathbb{R}^{3\times 3}$$ 中的旋转群。刚体的每一个姿态对应 SO(3) 中唯一一个矩阵；姿态随时间变化就是一条曲线 $$R(t) \in SO(3)$$。我们把 SO(3) 称为该系统的**配置空间 (configuration space)**。

旋转矩阵还有两重身份：它既是"姿态的坐标表示"，也是"把坐标从 B 系变换到 A 系"的线性映射：

$$q_a = R_{ab}\, q_b$$

向量的变换同样是旋转：若 $$v_b = q_b - p_b$$，则 $$R_{ab}(v_b) = R_{ab} q_b - R_{ab} p_b = v_a$$。三个坐标系连续嵌套时，旋转按"左乘"复合：

$$R_{ac} = R_{ab} R_{bc}$$

这是旋转的组合规则，也是后文一切复合操作（欧拉角、四元数、DH 参数）的逻辑根源。

进一步地，旋转矩阵是"刚性体变换"：它保距离、保定向。保距离是因为 $$\lVert Rq - Rp\rVert^2 = (q-p)^\top R^\top R (q-p) = \lVert q-p\rVert^2$$；保定向是因为旋转与叉积可交换，$$R(v \times w) = Rv \times Rw$$。这两条正好对应第 1 章"刚性运动"的定义，说明纯旋转确实是刚性运动的一种特殊情形。反过来，任意一个满足正交性与行列式 +1 的矩阵也必然对应某个真实的旋转——这就是为什么"用矩阵研究姿态"是完备的，不会漏掉任何可能的姿态。

### 2.3 反对称矩阵与 so(3)

叉积 $$a \times b$$ 是线性运算，可以写成矩阵乘：对任意向量 $$a \in \mathbb{R}^3$$，定义反对称矩阵

$$[a] = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}, \qquad a \times b = [a]\, b$$

所有 3×3 反对称矩阵构成向量空间：

$$so(3) = \{S \in \mathbb{R}^{3\times 3} : S^\top = -S\}$$

so(3) 与 $$\mathbb{R}^3$$ 通过 $$a \mapsto [a]$$ 一一对应。它就是后面指数映射的"定义域"——**so(3) 里的元素解释为旋转轴，SO(3) 里的元素解释为旋转结果**。

### 2.4 其他表示

除了旋转矩阵，还有三种常用表示：

- **轴角 / 指数坐标**：用单位轴 $$\omega$$ 和转角 $$\theta$$ 表示（欧拉定理：任意姿态等价于绕某固定轴转一个角度）；
- **欧拉角 (Euler angles)**：三次绕坐标轴（内旋）的旋转序列，如 ZYZ 或 ZYX（yaw-pitch-roll）；
- **四元数 (quaternion)**：四个数构成的超复数，单位四元数全局、无奇异地表示 SO(3)。

三种表示各有取舍，详见第 3 节。

## 3. 算法与公式

### 3.1 指数坐标与指数映射

考虑刚体以单位角速度绕单位轴 $$\omega$$ 旋转，其上一点 $$q(t)$$ 的速度为

$$\dot{q}(t) = \omega \times q(t) = [\omega]\, q(t)$$

这是一个时不变线性微分方程，解为 $$q(t) = e^{[\omega]t} q(0)$$，其中矩阵指数按幂级数定义：

$$e^{[\omega]\theta} = I + [\omega]\theta + \frac{[\omega]^2\theta^2}{2!} + \frac{[\omega]^3\theta^3}{3!} + \cdots$$

于是"绕 $$\omega$$ 轴转 $$\theta$$ 角"对应的旋转矩阵正是矩阵指数（指数映射）：

$$R = e^{[\omega]\theta}$$

幂级数无法直接用于计算，需要闭式解。利用反对称矩阵的幂次递推关系：

$$[\omega]^2 = \omega\omega^\top - \lVert\omega\rVert^2 I, \qquad [\omega]^3 = -\lVert\omega\rVert^2 [\omega]$$

对 $$\lVert\omega\rVert = 1$$ 的幂级数按奇偶项分组，就得到著名的**罗德里格斯公式 (Rodrigues' formula)**：

$$R = I + \sin\theta\, [\omega] + (1-\cos\theta)\, [\omega]^2, \qquad \lVert\omega\rVert = 1$$

当输入是未单位化的旋转向量 $$\theta \in \mathbb{R}^3$$（含长度 $$\lVert\theta\rVert$$）时，一般形式为

$$e^{[\hat{\omega}]\lVert\theta\rVert} = I + \frac{[\theta]}{\lVert\theta\rVert}\sin\lVert\theta\rVert + \frac{[\theta]^2}{\lVert\theta\rVert^2}\left(1 - \cos\lVert\theta\rVert\right)$$

这里的向量 $$\omega\theta$$（单位轴 × 角度）称为旋转的**指数坐标 (exponential coordinates)**。

从几何上看，$$[\omega]q = \omega \times q$$ 正是"绕 $$\omega$$ 轴以单位角速度转动时点 q 的瞬时速度"，因此反对称矩阵与角速度向量天然一一对应。这也是为什么称 so(3) 为"李代数"：它编码的是无穷小旋转（角速度、角增量），而指数映射负责把无穷小旋转"累积"成有限旋转。工程上，角速度积分、姿态估计、误差态卡尔曼滤波（ESKF）都建立在这条对应关系上；第 7 章的动力学中，角速度与惯性张量的耦合也正是在 so(3) 的框架下书写的。

### 3.2 对数映射：从矩阵提取轴角

指数映射 $$\exp : so(3) \to SO(3)$$ 是满射：任意旋转矩阵都能写成某个 $$e^{[\omega]\theta}$$。构造性求法即对数映射：

$$\theta = \cos^{-1}\left(\frac{\mathrm{trace}(R) - 1}{2}\right), \qquad \omega = \frac{1}{2\sin\theta}\begin{bmatrix} r_{32} - r_{23} \\ r_{13} - r_{31} \\ r_{21} - r_{12} \end{bmatrix}$$

注意两条：其一，$$\theta$$ 与 $$-\theta$$、$$\omega$$ 与 $$-\omega$$ 给出同一旋转，映射是"多对一"的，故轴角不唯一；其二，当 $$R = I$$ 时 $$\theta = 0$$，轴可任取，这是轴角表示的一个**奇异位形**（平滑性丢失）。伪代码：

```
function log_so3(R):
    θ ← acos((trace(R) − 1) / 2)
    if θ ≈ 0:  return 零向量
    return (θ / (2·sinθ)) · [r32−r23, r13−r31, r21−r12]ᵀ
```

实现对数映射时有几个数值细节值得注意：其一，当 $$\theta$$ 很小时 $$\sin\theta$$ 趋于零，式中的 $$1/(2\sin\theta)$$ 会放大浮点误差，工程上常用数值稳定的求法（如对旋转矩阵的反对称部分做缩放，或改用四元数提取）；其二，$$\theta = \pi$$ 附近矩阵对角元趋于退化，轴的提取也不稳定，需要按对角元取最大的分量来选主元；其三，$$\cos^{-1}$$ 的参数应夹取到 $$[-1,1]$$，避免浮点舍入导致数学域错误。

### 3.3 欧拉角

ZYZ 欧拉角：从 A 系出发，先绕（与 A 重合的）B 系 z 轴转 $$\alpha$$，再绕新的 y 轴转 $$\beta$$，再绕最新的 z 轴转 $$\gamma$$。三次基本旋转矩阵为

$$R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}, \quad R_y(\beta) = \begin{bmatrix} \cos\beta & 0 & \sin\beta \\ 0 & 1 & 0 \\ -\sin\beta & 0 & \cos\beta \end{bmatrix}, \quad R_z(\alpha) = \begin{bmatrix} \cos\alpha & -\sin\alpha & 0 \\ \sin\alpha & \cos\alpha & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

合成旋转为

$$R_{ab} = R_z(\alpha)\, R_y(\beta)\, R_z(\gamma)$$

（内旋序列按左乘复合。）反过来，由 $$R$$ 求角用 atan2 分象限：

$$\beta = \mathrm{atan2}\left(\sqrt{r_{31}^2 + r_{32}^2},\, r_{33}\right), \qquad \alpha = \mathrm{atan2}\left(\frac{r_{23}}{\sin\beta},\, \frac{r_{13}}{\sin\beta}\right), \qquad \gamma = \mathrm{atan2}\left(\frac{r_{32}}{\sin\beta},\, -\frac{r_{31}}{\sin\beta}\right)$$

当 $$\sin\beta \approx 0$$（即 $$\beta \approx 0$$ 或 $$\pi$$）时上述公式失效，这就是**万向节锁 (gimbal lock)**。这是一个拓扑事实：SO(3) 是三维紧流形，**任何三维坐标表示都必然存在奇异点**，无法用三个数全局光滑地覆盖它。欧拉角是"局部参数化"。

### 3.4 四元数

四元数是 $$Q = q_0 + q_1 i + q_2 j + q_3 k$$，简记 $$Q = (q_0, \vec{q})$$。乘法满足 $$i\cdot j = k$$ 等规则，不可交换。两四元数乘积可写成点积与叉积形式（四元数乘法）：

$$Q \cdot P = (q_0 p_0 - \vec{q}\cdot\vec{p},\, q_0\vec{p} + p_0\vec{q} + \vec{q}\times\vec{p})$$

这是实现四元数乘法唯一需要的公式（不必手写 ijk 乘法表）。单位四元数（$$\lVert Q\rVert = 1$$）与旋转一一对应（除符号：$$Q$$ 与 $$-Q$$ 是同一旋转，故单位四元数构成 SO(3) 的"双覆盖"）。轴角到四元数：

$$Q = \left(\cos\frac{\theta}{2},\, \omega\sin\frac{\theta}{2}\right)$$

复合旋转直接做四元数乘法：$$Q_{ac} = Q_{ab}\cdot Q_{bc}$$。旋转向量 $$v$$：$$v' = Q\, v\, Q^{*}$$（$$Q^{*} = (q_0, -\vec{q})$$ 为共轭，单位四元数下即逆）。四元数转旋转矩阵（令 $$Q = (w,x,y,z)$$）：

$$R = \begin{bmatrix} 1-2(y^2+z^2) & 2(xy - wz) & 2(xz + wy) \\ 2(xy + wz) & 1-2(x^2+z^2) & 2(yz - wx) \\ 2(xz - wy) & 2(yz + wx) & 1-2(x^2+y^2) \end{bmatrix}$$

与欧拉角相比，四元数用 4 个数换来**全局无奇异**，是 SLAM、姿态估计、动画插值（slerp）的首选。

## 4. C 语言实现

本节给出两段可独立编译的 C11 代码。第一段实现四元数的乘法、归一化、轴角构造、旋转向量与转矩阵；第二段实现罗德里格斯公式与 so(3) 指数映射。均只依赖标准库 `math.h`，手写 3×3 矩阵与向量运算。

### 4.1 四元数

```c
/* 文件: ch03_quat.c — 四元数表示三维旋转
 * 编译: gcc -std=c11 -lm -Wall -Wextra ch03_quat.c -o quat_demo
 */
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { double x, y, z; } Vec3;   /* 三维向量 */
typedef struct { double w, x, y, z; } Quat; /* 四元数: w 标量, (x,y,z) 向量 */
typedef struct { double m[3][3]; } Mat3;    /* 3x3 旋转矩阵 */

static double vec_dot(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static Vec3 vec_cross(Vec3 a, Vec3 b) {     /* 叉积 a×b */
    Vec3 v = {a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x};
    return v;
}

/* 四元数乘法 (MLS 公式): Q·P = (q0 p0 − q·p, q0 p + p0 q + q×p) */
static Quat quat_mul(Quat q, Quat p) {
    Vec3 qv = {q.x, q.y, q.z}, pv = {p.x, p.y, p.z};
    Vec3 c  = vec_cross(qv, pv);
    Quat r;
    r.w = q.w*p.w - vec_dot(qv, pv);
    r.x = q.w*pv.x + p.w*qv.x + c.x;
    r.y = q.w*pv.y + p.w*qv.y + c.y;
    r.z = q.w*pv.z + p.w*qv.z + c.z;
    return r;
}

/* 归一化: 使四元数成为单位四元数 */
static Quat quat_normalize(Quat q) {
    double n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    Quat r = {q.w/n, q.x/n, q.y/n, q.z/n};
    return r;
}

/* 轴角(单位轴 omega, 角 theta) → 四元数 */
static Quat quat_from_axis_angle(Vec3 omega, double theta) {
    double s = sin(theta/2.0);
    Quat q = {cos(theta/2.0), s*omega.x, s*omega.y, s*omega.z};
    return quat_normalize(q);
}

/* 用单位四元数旋转向量: v' = q v q* (共轭即逆) */
static Vec3 quat_rotate(Quat q, Vec3 v) {
    Quat vq = {0.0, v.x, v.y, v.z};
    Quat qc = {q.w, -q.x, -q.y, -q.z};
    Quat t  = quat_mul(q, vq);
    Quat r  = quat_mul(t, qc);
    Vec3 out = {r.x, r.y, r.z};
    return out;
}

/* 单位四元数 → 旋转矩阵 (逐元素赋值, 避免嵌套花括号初始化) */
static Mat3 quat_to_mat3(Quat q) {
    double w=q.w, x=q.x, y=q.y, z=q.z;
    Mat3 R;
    R.m[0][0]=1-2*(y*y+z*z); R.m[0][1]=2*(x*y-w*z);   R.m[0][2]=2*(x*z+w*y);
    R.m[1][0]=2*(x*y+w*z);   R.m[1][1]=1-2*(x*x+z*z); R.m[1][2]=2*(y*z-w*x);
    R.m[2][0]=2*(x*z-w*y);   R.m[2][1]=2*(y*z+w*x);   R.m[2][2]=1-2*(x*x+y*y);
    return R;
}

int main(void) {
    /* 绕 z 轴转 90°, 旋转向量 (1,0,0) */
    Vec3 axis = {0.0, 0.0, 1.0};
    Quat q  = quat_from_axis_angle(axis, M_PI/2.0);
    Vec3 v  = {1.0, 0.0, 0.0};
    Vec3 vp = quat_rotate(q, v);
    printf("绕 z 转 90°: (%.4f, %.4f, %.4f)  (期望约 0,1,0)\n",
           vp.x, vp.y, vp.z);

    Mat3 R = quat_to_mat3(q);
    printf("旋转矩阵 R:\n");
    for (int i = 0; i < 3; ++i)
        printf("  %.4f %.4f %.4f\n", R.m[i][0], R.m[i][1], R.m[i][2]);

    /* 复合: 先绕 z 转 30°, 再绕 x 转 45° */
    Vec3 az = {0,0,1}, ax = {1,0,0};
    Quat q1 = quat_from_axis_angle(az, M_PI/6.0);
    Quat q2 = quat_from_axis_angle(ax, M_PI/4.0);
    Quat qc = quat_mul(q2, q1);           /* 先 q1 后 q2 */
    Vec3 w  = {1, 0, 0};
    Vec3 wp = quat_rotate(qc, w);
    printf("复合旋转: (%.4f, %.4f, %.4f)\n", wp.x, wp.y, wp.z);
    return 0;
}
```

### 4.2 罗德里格斯公式与指数映射

```c
/* 文件: ch03_rodrigues.c — 罗德里格斯公式 + so(3) 指数映射
 * 编译: gcc -std=c11 -lm -Wall -Wextra ch03_rodrigues.c -o rodrigues
 */
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { double m[3][3]; } Mat3;

/* 由向量构造反对称矩阵 (叉积矩阵): [omega] */
static void skew(const double omega[3], Mat3 *S) {
    S->m[0][0]=0.0;      S->m[0][1]=-omega[2]; S->m[0][2]= omega[1];
    S->m[1][0]= omega[2]; S->m[1][1]=0.0;       S->m[1][2]=-omega[0];
    S->m[2][0]=-omega[1]; S->m[2][1]= omega[0]; S->m[2][2]=0.0;
}

/* 3x3 矩阵乘: C = A*B (行主序) */
static void mat3_mul(const Mat3 *A, const Mat3 *B, Mat3 *C) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += A->m[i][k]*B->m[k][j];
            C->m[i][j] = s;
        }
}

/* 罗德里格斯公式: 单位轴 omega, 角 theta
 * R = I + sinθ·[ω] + (1−cosθ)·[ω]^2 */
static void rodrigues(const double omega[3], double theta, Mat3 *R) {
    double c = cos(theta), s = sin(theta), v = 1.0 - c;
    Mat3 W, W2;
    skew(omega, &W);
    mat3_mul(&W, &W, &W2);            /* W2 = [ω]^2 */
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R->m[i][j] = (i==j ? 1.0 : 0.0) + s*W.m[i][j] + v*W2.m[i][j];
}

/* 指数映射: 输入旋转向量 tv[3] = ω·θ (不必单位化)
 * R = exp([ω]θ): 先归一化出单位轴, 再调 rodrigues */
static void exp_so3(const double tv[3], Mat3 *R) {
    double th = sqrt(tv[0]*tv[0] + tv[1]*tv[1] + tv[2]*tv[2]);
    if (th < 1e-12) {                 /* 零旋转 → 单位阵 */
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R->m[i][j] = (i==j ? 1.0 : 0.0);
        return;
    }
    double w[3] = {tv[0]/th, tv[1]/th, tv[2]/th};
    rodrigues(w, th, R);
}

int main(void) {
    /* 例 1: 绕 z 轴转 π/2, 旋转向量 (0,0,π/2) */
    double tv[3] = {0.0, 0.0, M_PI/2.0};
    Mat3 R;
    exp_so3(tv, &R);
    printf("exp([z·π/2]):\n");
    for (int i = 0; i < 3; ++i)
        printf("  %.4f %.4f %.4f\n", R.m[i][0], R.m[i][1], R.m[i][2]);

    /* 例 2: 通用轴角, 轴 (1,1,0)/√2 绕 60°
     * 自检: trace(R) 应等于 1+2·cos(60°) = 2 */
    double a = 1.0/sqrt(2.0);
    double w[3] = {a, a, 0.0};
    Mat3 R2;
    rodrigues(w, M_PI/3.0, &R2);
    double tr = R2.m[0][0] + R2.m[1][1] + R2.m[2][2];
    printf("trace = %.6f (期望 2.000000)\n", tr);

    /* 例 3: 对例 1 的矩阵做对数映射自检: 应还原转角 θ=π/2 */
    double tr3 = R.m[0][0] + R.m[1][1] + R.m[2][2];
    double th2 = acos((tr3 - 1.0)/2.0);
    printf("对数映射反解: θ = %.4f (期望 1.5708≈π/2)\n", th2);
    return 0;
}
```

代码说明：`rodrigues` 要求输入单位轴，`exp_so3` 负责把任意旋转向量归一化并分发；例 3 演示了对数映射的粗糙自检——用 $$\mathrm{acos}\big((\mathrm{trace}(R)-1)/2\big)$$ 反解转角，验证指数映射实现了欧拉定理中的等价轴表示。

## 5. 教材/论文精读

本段精读 MLS 第 2 章 1-3 节（原书 2.1-2.3，对应文本 23-34 页）。

**2.1 节（旋转矩阵与 SO(3)）的核心思想**是把"姿态"彻底代数化：刚体相对于惯性系的姿态被编码为三个体轴坐标向量拼成的矩阵 $$R_{ab}$$，其正交性 + 行列式 +1 两条性质完全刻画了"右手正交标架"这一几何对象，从而把旋转问题转化为矩阵代数问题。教科书随后严格验证 SO(3) 满足群的四条公理，并证明旋转保距、保定向（旋转与叉积可交换：$$R(v \times w) = Rv \times Rw$$）。这一节的方法论启示在于：**用"保持结构不变的变换集合 + 群结构"来建模系统**，是后面 SE(3)、李群方法的模板。

**2.2 节（指数坐标）**是最具思想深度的一节。它不从矩阵拼接出发，而是从运动学事实反推：绕定轴匀速旋转的点的速度是 $$\dot{q} = [\omega]q$$，这是线性微分方程，解自然给出矩阵指数 $$e^{[\omega]t}$$。于是旋转被重新解释为"李代数 so(3) 中元素经指数映射生成的李群 SO(3) 元素"。幂级数经过反对称矩阵的幂次递推（$$[\omega]^3 = -\lVert\omega\rVert^2[\omega]$$）化简为三项闭式——罗德里格斯公式。反向的对数映射则给出从任意旋转矩阵提取轴角的构造性算法。这里的启示：**把"有限旋转"放进"无穷小旋转（角速度）+ 积分"的框架**，不仅计算上得到闭式，还直接连接了后面的旋量（twist）与乘积指数公式（PoE）。

**2.3 节（其他表示）**用指数坐标作参照系，点评了欧拉角与四元数。欧拉角的 ZYZ 公式（教材式 2.19）与提取公式（式 2.20）体现了"三次内旋 = 三个基本旋转左乘"，其奇异（万向节锁）本质上是**拓扑必然**：SO(3) 是三维紧流形，任何三维全局光滑参数化都不存在。四元数则用 4 个参数换来全局无奇异，代价是多一个约束（单位范数）与一点冗余。MLS 给出轴角到四元数、四元数乘法、四元数到矩阵的完整对应，并强调四元数与旋转群同构、适合插值。局限方面：教材对数值实现（如四元数规范化漂移、atan2 分支、$$\theta$$ 的 ±2π 歧义）着墨不多，这些在工程实现时需自行补齐。

## 6. 实验/编程作业

对应 EECS C106A 第 2 周，建议按以下顺序动手：

1. **表示互转**：用 C 或 Python 写轴角、欧拉角（ZYZ/ZYX）、四元数、旋转矩阵四种表示之间的双向转换，并用随机轴角验证往返误差（先 A 到 B 再回到 A，检查与单位阵的偏差）。
2. **复合一致性**：随机生成两个旋转 $$R_1, R_2$$，验证矩阵乘法复合、四元数乘法复合、欧拉角序列复合三者结果一致（数值误差小于 1e-8）。
3. **对数映射自检**：对随机旋转矩阵做 log_so3 再 exp_so3，检查还原误差；特别测试 $$R = I$$ 与 $$\theta = \pi$$ 两个退化情形。
4. **实现你自己的旋转工具包**：把本章两段 C 代码合并扩展成 `rot.h`/`rot.c`，加入欧拉角互转与 slerp 球面插值——后续第 4 章 SE(3)、第 5 章正运动学会直接复用。

若使用 ROS，可用 `tf2` 与 `tf_transformations.py` 交叉验证你的实现；养成"所有旋转都先统一成四元数或旋转矩阵再运算"的习惯，可避免欧拉角万向节锁带来的调试麻烦。

## 7. 延伸阅读

- 官方教材：Murray, Li, Sastry，《A Mathematical Introduction to Robotic Manipulation》，第 2 章（2.1-2.3 节），`pdfs/MLS_textbook.txt`。
- EECS C106A《Introduction to Robotics》（UC Berkeley）第 2 周讲义与作业。
- 视觉 SLAM 中四元数的实战讲解：Sola, "Quaternion kinematics for the error-state Kalman filter"（arXiv:1711.02508）。
- 开源实现：Eigen 几何模块（`Eigen::Quaterniond`、`Eigen::AngleAxis`）、ROS tf2、Python `scipy.spatial.transform.Rotation`。
- 四元数与插值：Shoemake, "Animating rotation with quaternion curves"（SIGGRAPH 1985），slerp 出处。

## 8. 小结

本章建立了描述纯旋转的三套语言：旋转矩阵与 SO(3) 群、指数坐标与罗德里格斯公式、欧拉角与四元数。旋转矩阵是"姿态的规范坐标"，SO(3) 是它的群结构；指数映射把旋转向量变成旋转矩阵，罗德里格斯公式给出闭式计算，对数映射负责反向提取；欧拉角直观但局部有奇异，四元数全局无奇异但多一维。四种表示间的互转是本章的实践核心，两段 C 代码已给出四元数与罗德里格斯公式的完整实现。下一章，我们把旋转与平移合成为 SE(3) 齐次变换，正式迈向完整刚体运动学。

---
[上一章](ch02_线性代数与数学基础.md) ｜ [下一章](ch04_刚性体变换与SE3.md)
