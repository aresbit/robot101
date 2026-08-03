# 第 4 章 刚性体变换与 SE(3)

> 课程来源：EECS C106A 第 2-3 周 ｜ 关键词：齐次变换、SE(3) 群、旋量 twist、螺旋 screw、指数坐标 ｜ 预计阅读时长：~25 分钟

## 1. 导论

第 3 章只处理了旋转：SO(3) 上的群结构、罗德里格斯公式与指数坐标刻画的是刚体**绕坐标原点转动**的姿态。但真实机器人的连杆不仅有姿态，还有位置——两个关节之间的连杆存在平移偏移，末端执行器相对基座的位置与姿态必须用一个同时容纳旋转与平移的对象来描述。本章把运动学从"只旋转"推广到"既旋转又平移"的完整**刚性运动 (rigid motion)**。

完整刚性运动的配置空间是**特殊欧氏群** SE(3)（Special Euclidean Group），它是 SO(3) 与平动空间 R3 的半直积。本章将依次建立三套互等价的工具：

1. **齐次表示 (homogeneous representation)**：把旋转变换矩阵 R 与平移向量 p 拼成一个 4x4 矩阵，使复合与求逆都归结为普通矩阵乘法，代价是把维度从 3 提升到 4。
2. **旋量 (twist)**：把刚体瞬时运动压缩成一个 6 维向量，其 4x4 矩阵形式落在李代数 se(3) 上，是 SE(3) 的"无穷小生成元"。
3. **指数坐标 (exponential coordinates)**：每一个刚性变换都能写成某个旋量的矩阵指数的形式，即 $$g = e^{\hat{\xi}\theta}$$，从而把连续运动与离散位姿统一。

从维数的角度看，SE(3) 是 6 维流形：3 个平移自由度加 3 个转动自由度。这正是操作臂末端执行器在工作空间中占据的完整配置空间，也是视觉伺服、柔顺控制等任务层面算法的天然坐标系。相比之下，第 3 章的 SO(3) 只刻画姿态的 3 个自由度；把位置与姿态分开处理会丢失二者在运动链中的耦合结构——例如旋转关节的转轴会随前序连杆的平移而移动。SE(3) 的统一表示正是为保留这种耦合而生。

这套框架的直接回报是第 5 章的**乘积指数公式 (Product of Exponentials, PoE)**：机器人的正运动学只需要把各关节旋量的指数按顺序相乘，无需 DH 参数化。EECS C106A 在第 2-3 周完成从 SO(3) 到 SE(3) 的过渡，并在这里引入旋量语言，为后续以指数坐标为核心的运动学、速度与动力学分析铺路。

## 2. 关键概念与记号

### 2.1 配置与 SE(3)

设坐标系 B 固连于刚体，惯性系 A 为世界系。刚体的**配置 (configuration)** 由一对数据给出：原点位置向量 $$p_{ab} \in \mathbb{R}^3$$ 与姿态矩阵 $$R_{ab} \in SO(3)$$。所有可能配置构成的集合记作

$$
SE(3) = \{(p, R) : p \in \mathbb{R}^3,\; R \in SO(3)\} = \mathbb{R}^3 \times SO(3).
$$

一个元素 $$g = (p, R) \in SE(3)$$ 身兼二职：既描述刚体的配置，又作为一个**变换**把点在体坐标系 B 与惯性系 A 之间换系：

$$
q_a = p_{ab} + R_{ab}\, q_b.
$$

注意**点**按上式变换（旋转加平移），而**向量**（两点之差）只做旋转变换：$$g_*(v) = R v$$。

### 2.2 齐次表示与群公理

为把仿射变换 $$q_a = R q_b + p$$ 变成线性矩阵乘法，我们给点追加一个分量 1、给向量追加分量 0，得到 R4 中的**齐次坐标**。于是刚性变换可写成 4x4 矩阵

$$
g = \begin{pmatrix} R & p \\ 0 & 1 \end{pmatrix}.
$$

齐次坐标末尾分量 0/1 强制执行四条"语法规则"：向量加向量得向量；向量加点得点；点减点得向量；点加点无意义。齐次矩阵的末行 `0 0 0 1` 看似多余，但正是它把仿射变换线性化。图形学中把末行替换为透视投影向量、或把右下角标量换成大于 1 的数，可产生透视或缩放效果，但那时矩阵已不再对应刚体运动——这提醒我们 SE(3) 的刚性约束（R 为正交矩阵、末行为 `0 0 0 1`）必须被严格保留。这个记号让 SE(3) 的群结构一目了然：

- **封闭性**：若 $$g_1, g_2 \in SE(3)$$，则矩阵乘积 $$g_1 g_2 \in SE(3)$$；
- **单位元**：4x4 单位阵 $$I_4 \in SE(3)$$；
- **逆元**：$$\begin{pmatrix} R & p \\ 0 & 1 \end{pmatrix}^{-1} = \begin{pmatrix} R^T & -R^T p \\ 0 & 1 \end{pmatrix}$$，即 $$g^{-1} = (-R^T p, R^T)$$；
- **结合律**：矩阵乘法天然满足。

复合规则 $$g_{ac} = g_{ab} g_{bc}$$ 对应沿链式坐标系依次换系，是后续正运动学的基石。

### 2.3 旋量 twist 与 se(3)

刚体的瞬时速度不能像质点那样直接对 g 求导——$$\dot{g} \notin SE(3)$$ 也不在某个向量空间中。正确的做法是考虑 $$\dot{g} g^{-1}$$ 或 $$g^{-1}\dot{g}$$，二者都落在李代数 se(3) 上：

$$
se(3) := \{(v, \hat{\omega}) : v \in \mathbb{R}^3,\; \hat{\omega} \in so(3)\},
\qquad
\hat{\xi} = \begin{pmatrix} \hat{\omega} & v \\ 0 & 0 \end{pmatrix} \in \mathbb{R}^{4 \times 4}.
$$

其中 $$\hat{\omega}$$ 是向量 $$\omega$$ 的反对称矩阵（hat 映射）。6 维坐标 $$(\hat{\xi})^\vee = (v, \omega) =: \xi \in \mathbb{R}^6$$ 称为**旋量坐标 (twist coordinates)**：$$\omega$$ 是角速度/转轴方向，$$v$$ 是线性部分。对应地，wedge 算子把 6 维向量重新拼成 4x4 矩阵。

一个常被问的问题是：为什么速度只需要 6 个数而 4x4 矩阵有 16 个分量？答案是旋量去掉了 SE(3) 中由群结构决定的冗余。正交条件 $$R^T R = I$$ 与单位行列式把旋转矩阵从 9 个数压缩到 3 个独立自由度，平移再贡献 3 个，合计 6 个——这正是李代数维数与流形维数相等的体现。在纯转动情形，旋量满足 $$v = -\omega \times q$$，因此只需给出转轴方向 $$\omega$$ 与轴上一点 $$q$$，六个分量便全部确定。

### 2.4 螺旋 screw

把旋量与其生成的刚体运动关联起来的几何对象是**螺旋 (screw)**：绕某条轴旋转 θ 同时沿同轴平移 d。定义

- **轴 (axis)**：方向为 ω 的一条直线，经过点 $$\frac{\omega \times v}{\lVert \omega \rVert^2}$$；
- **螺距 (pitch)**：$$h = \omega^T v / \lVert\omega\rVert^2$$，即平移量与旋转量之比；
- **模 (magnitude)**：$$M = \lVert\omega\rVert$$（ω 非零时取旋转量，否则取平移量）。

**Chasles 定理**：任何刚体运动都可实现为绕某轴旋转再沿该轴平移——即任意 SE(3) 元素都是某个螺旋运动。零螺距螺旋对应纯旋转（转轴关节 revolute joint 的模型），无穷螺距螺旋对应纯平移（移动关节 prismatic joint 的模型）。

### 2.5 空间速度、体速度与伴随

对轨迹 $$g(t) \in SE(3)$$：

- **空间速度 (spatial velocity)**：$$V^s = \dot{g} g^{-1}$$，物理意义是"经过空间系原点瞬间的刚体点速度"；
- **体速度 (body velocity)**：$$V^b = g^{-1}\dot{g}$$，物理意义更直观——体坐标系原点相对空间系的速度、姿态角速度，都投影到当前体坐标系。

二者通过 6x6 **伴随矩阵 (adjoint)** 联系：

$$
\mathrm{Ad}_g = \begin{pmatrix} R & \hat{p} R \\ 0 & R \end{pmatrix},
\qquad V^s = \mathrm{Ad}_g\, V^b.
$$

伴随矩阵还有一个更广的用途（MLS 2.64 式）：给定常值旋量 ξ，用刚性变换 g 移动其螺旋轴，得到新旋量 $$\xi' = \mathrm{Ad}_g \xi$$，这正是关节被连杆搬动后旋量轴的变化规则。

## 3. 算法与公式

### 3.1 齐次变换基本公式

点换系、复合与求逆：

$$
q_a = R_{ab} q_b + p_{ab},
\qquad
g_{ac} = g_{ab} g_{bc},
\qquad
g^{-1} = \begin{pmatrix} R^T & -R^T p \\ 0 & 1 \end{pmatrix}.
$$

### 3.2 旋量的指数映射

旋量 $$(\hat{\xi}, \theta)$$ 的矩阵指数给出从初始位形到末态位形的刚性运动：

$$
e^{\hat{\xi}\theta} = \begin{pmatrix} e^{\hat{\omega}\theta} & (I-e^{\hat{\omega}\theta})(\omega \times v) + \omega\omega^T v \theta \\ 0 & 1 \end{pmatrix}, \qquad \omega \neq 0.
$$

其中转角分量用罗德里格斯公式计算（$$\lVert\omega\rVert = 1$$）：

$$
e^{\hat{\omega}\theta} = I + \sin\theta\, \hat{\omega} + (1-\cos\theta)\, \hat{\omega}^2.
$$

**纯平移特例**（$$\omega = 0$$）：高阶项全部为零，只剩一阶项

$$
e^{\hat{\xi}\theta} = \begin{pmatrix} I & v\theta \\ 0 & 1 \end{pmatrix}.
$$

指数映射 $$exp: se(3) \to SE(3)$$ 是**满射**（每个刚体变换都能写成某个旋量的指数，MLS 命题 2.9），但不是单射——ω 与 θ 的选择不唯一，出现 2π 周期等价的歧义。

### 3.3 由螺旋/关节构造旋量

- **旋转关节**：轴方向 ω（单位向量），轴上一点 q，则旋量 $$v = -\omega \times q$$，即 $$\xi = (-\omega \times q, \omega)$$。
- **移动关节**：平动方向 v（单位向量），则旋量 $$\xi = (v, 0)$$。
- **一般螺旋**：轴经过 q、方向 ω、螺距 h，则 $$v = -\omega \times q + h\omega$$。

### 3.4 速度公式

空间/体速度及其变换：

$$
V^s = \dot{g}g^{-1},\qquad V^b = g^{-1}\dot{g},\qquad V^s = \mathrm{Ad}_g V^b.
$$

对螺旋运动 $$g(\theta) = e^{\hat{\xi}\theta} g(0)$$，空间速度恰好恒等于常值旋量 $$\dot{\theta}\xi$$，这解释了为何旋量被称为"单位速度下的螺旋运动"。

**数值例**：绕穿过点 $$q=(0,1,0)$$ 的 z 轴旋转 90 度。取 $$\omega=(0,0,1)$$，则 $$v = -\omega \times q = (1,0,0)$$，旋量为 $$\xi = ((1,0,0),(0,0,1))$$。罗德里格斯公式给出 $$e^{\hat{\omega}\theta} = \begin{pmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{pmatrix}$$；而 $$(I - e^{\hat{\omega}\theta})(\omega \times v) = (1,1,0)$$，又因为 $$\omega^T v = 0$$ 使第二项消失，故平移分量为 $$(1,1,0)$$。这与几何直觉一致：以 (0,1,0) 为圆心的单位半径圆盘绕 z 轴转 90 度后，体坐标点 (1,0,0) 到达 (1,2,0)。该数值演示会在第 4 节 C 代码的输出中复现。

## 4. C 语言实现

本节给出两段可独立编译的 C11 程序：第一段实现 SE(3) 齐次变换的代数运算；第二段实现旋量指数映射与伴随映射。风格遵循"手写 3x3/4x4 矩阵运算、不依赖外部线性代数库"的数值计算路线。

### 4.1 SE(3) 齐次变换：复合、求逆与点变换

```c
/* ch04_se3_basic.c
 * 编译运行: gcc -std=c11 ch04_se3_basic.c -o se3_basic -lm && ./se3_basic
 * 内容: struct SE3 = 旋转矩阵 R + 平移向量 p（即齐次矩阵 [[R, p], [0, 1]]）；
 *       se3_identity / se3_mul / se3_inv / se3_transform_point。
 * 演示: 构造 "先绕 z 轴旋转 90 度、再沿世界系 y 轴平移 2" 的复合变换，
 *       变换一个体坐标点，并用逆变换还原；最后校验旋转部分正交性。
 */
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

typedef struct { double x, y, z; } Vec3;   /* 三维向量 */
typedef struct { double m[3][3]; } Mat3;   /* 3x3 矩阵  */
typedef struct { Mat3 R; Vec3 p; } SE3;    /* 齐次变换   */

static Vec3 v3(double x, double y, double z) { Vec3 v = {x, y, z}; return v; }
static Vec3 v3_add(Vec3 a, Vec3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static Vec3 v3_scale(Vec3 a, double s) { return v3(a.x*s, a.y*s, a.z*s); }
static Vec3 v3_cross(Vec3 a, Vec3 b) {
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static double v3_dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

static Mat3 m3_zero(void) {           /* 用循环清零，避开聚合初始化 */
    Mat3 m;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m.m[i][j] = 0.0;
    return m;
}
static Mat3 m3_identity(void) {
    Mat3 m = m3_zero();
    m.m[0][0] = m.m[1][1] = m.m[2][2] = 1.0;
    return m;
}
static Mat3 m3_mul(Mat3 a, Mat3 b) {  /* 手写 3x3 矩阵乘法 */
    Mat3 r = m3_zero();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}
static Vec3 m3_apply(Mat3 m, Vec3 v) { /* 矩阵乘向量 */
    return v3(m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z,
              m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z,
              m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z);
}
static Mat3 rot_z(double th) {        /* 绕 z 轴旋转 */
    Mat3 m = m3_identity();
    double c = cos(th), s = sin(th);
    m.m[0][0] = c; m.m[0][1] = -s;
    m.m[1][0] = s; m.m[1][1] = c;
    return m;
}

static SE3 se3_identity(void) { SE3 g = {m3_identity(), v3(0,0,0)}; return g; }

/* 齐次矩阵乘法: [[R1,p1],[0,1]] * [[R2,p2],[0,1]] = [[R1R2, R1p2+p1],[0,1]] */
static SE3 se3_mul(SE3 a, SE3 b) {
    SE3 g;
    g.R = m3_mul(a.R, b.R);
    g.p = v3_add(m3_apply(a.R, b.p), a.p);
    return g;
}

/* 逆变换: g^{-1} = (R^T, -R^T p) */
static SE3 se3_inv(SE3 g) {
    SE3 gi;
    for (int i = 0; i < 3; i++)           /* 手写转置 */
        for (int j = 0; j < 3; j++)
            gi.R.m[i][j] = g.R.m[j][i];
    gi.p = v3_scale(m3_apply(gi.R, g.p), -1.0);
    return gi;
}

/* 变换点: g(q) = R q + p */
static Vec3 se3_transform_point(SE3 g, Vec3 q) {
    return v3_add(m3_apply(g.R, q), g.p);
}

int main(void) {
    SE3 g1 = {rot_z(PI / 2), v3(0, 2, 0)};   /* 绕 z 轴 90 度 + 平移到 (0,2,0) */
    SE3 g2 = {rot_z(0),      v3(1, 0, 0)};   /* 纯平移 (1,0,0)                 */
    SE3 g  = se3_mul(g1, g2);                /* 复合: g = g1 * g2               */
    SE3 gi = se3_inv(g);
    SE3 gid = se3_mul(g, se3_identity());    /* 右乘单位元应保持 g 不变        */

    Vec3 q  = v3(1, 0, 0);                   /* 体坐标系中的点 */
    Vec3 qa = se3_transform_point(g, q);     /* 映射到世界系   */
    Vec3 qr = se3_transform_point(gi, qa);   /* 逆变换应还原   */

    printf("g = g1*g2 (Rz90 后平移):\n");
    printf("  点 (1,0,0) -> (%.4f, %.4f, %.4f)\n", qa.x, qa.y, qa.z);
    printf("  逆变换还原  -> (%.4f, %.4f, %.4f)\n", qr.x, qr.y, qr.z);
    printf("  g*g^{-1} 平移部分 (应为 0,0,0): (%.4f, %.4f, %.4f)\n",
           se3_mul(g, gi).p.x, se3_mul(g, gi).p.y, se3_mul(g, gi).p.z);
    printf("  g*I 与 g 平移差 (应为 0,0,0): (%.4f, %.4f, %.4f)\n",
           gid.p.x - g.p.x, gid.p.y - g.p.y, gid.p.z - g.p.z);

    /* 校验旋转部分正交性: 列1·列2 = 0, 列1×列2 = 列3 */
    Vec3 c1 = v3(g.R.m[0][0], g.R.m[1][0], g.R.m[2][0]);
    Vec3 c2 = v3(g.R.m[0][1], g.R.m[1][1], g.R.m[2][1]);
    Vec3 c3 = v3(g.R.m[0][2], g.R.m[1][2], g.R.m[2][2]);
    Vec3 cr = v3_cross(c1, c2);
    printf("正交性: 列1·列2 = %.6f, 列1×列2-列3 = (%.6f, %.6f, %.6f)\n",
           v3_dot(c1, c2), cr.x-c3.x, cr.y-c3.y, cr.z-c3.z);
    return 0;
}
```

### 4.2 旋量指数映射 exp_se3 与伴随映射

```c
/* ch04_twist_exp.c
 * 编译运行: gcc -std=c11 ch04_twist_exp.c -o twist_exp -lm && ./twist_exp
 * 理论: MLS 2.3 节式 (2.36)。对单位方向 ω 与线性部分 v:
 *   e^{ξθ} = [[ e^{ωθ},   (I - e^{ωθ})(ω×v) + ωω^T v θ ], [ 0, 1 ]]
 *   当 ω = 0 时退化为纯平移: e^{ξθ} = [[ I, vθ ], [ 0, 1 ]]
 * 内容: struct Twist 旋量坐标 ξ=(v,ω)∈R^6；hat() 构造反对称矩阵；
 *       exp_se3() 螺旋转动指数映射；adjoint() 伴随映射；
 *       revolute_twist() 由轴方向 ω 与轴上一点 q 构造旋转关节旋量 v=-ω×q。
 */
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

typedef struct { double x, y, z; } Vec3;
typedef struct { double m[3][3]; } Mat3;
typedef struct { Mat3 R; Vec3 p; } SE3;
typedef struct { Vec3 v; Vec3 w; } Twist;   /* 旋量坐标 ξ = (v, ω) */

static Vec3 v3(double x, double y, double z) { Vec3 v = {x, y, z}; return v; }
static Vec3 v3_add(Vec3 a, Vec3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static Vec3 v3_scale(Vec3 a, double s) { return v3(a.x*s, a.y*s, a.z*s); }
static double v3_dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static Vec3 v3_cross(Vec3 a, Vec3 b) {
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static Mat3 m3_zero(void) {
    Mat3 m;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m.m[i][j] = 0.0;
    return m;
}
static Mat3 m3_identity(void) {
    Mat3 m = m3_zero();
    m.m[0][0] = m.m[1][1] = m.m[2][2] = 1.0;
    return m;
}
static Mat3 m3_add(Mat3 a, Mat3 b) {
    Mat3 r = m3_zero();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = a.m[i][j] + b.m[i][j];
    return r;
}
static Mat3 m3_scale(Mat3 a, double s) {
    Mat3 r = m3_zero();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = a.m[i][j] * s;
    return r;
}
static Vec3 m3_apply(Mat3 m, Vec3 v) {
    return v3(m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z,
              m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z,
              m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z);
}

/* hat 映射: 三维向量 ω -> 反对称矩阵 ω̂，满足 ω̂ u = ω × u */
static Mat3 hat(Vec3 w) {
    Mat3 m = m3_zero();
    m.m[0][1] = -w.z; m.m[0][2] =  w.y;
    m.m[1][0] =  w.z; m.m[1][2] = -w.x;
    m.m[2][0] = -w.y; m.m[2][1] =  w.x;
    return m;
}

/* 罗德里格斯公式: e^{ω̂θ} = I + sin(θ)ω̂ + (1-cos(θ))ω̂², 要求 ‖ω‖ = 1 */
static Mat3 rodrigues(Vec3 w, double th) {
    Mat3 Wh  = hat(w);
    Mat3 Wh2 = m3_zero();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                Wh2.m[i][j] += Wh.m[i][k] * Wh.m[k][j];
    return m3_add(m3_add(m3_identity(), m3_scale(Wh, sin(th))),
                  m3_scale(Wh2, 1.0 - cos(th)));
}

/* 旋量指数映射 (MLS 式 2.36)。w 为 ω 方向向量(内部单位化), v 为线性部分,
 * th 为标量 θ；当 ω≈0 时退化为纯平移。 */
static SE3 exp_se3(Vec3 w, Vec3 v, double th) {
    SE3 g;
    double n = sqrt(v3_dot(w, w));
    if (n < 1e-12) {                       /* 纯平移: ω = 0 */
        g.R = m3_identity();
        g.p = v3_scale(v, th);
        return g;
    }
    Vec3 wu = v3_scale(w, 1.0 / n);        /* 单位化 ω          */
    double a = n * th;                     /* 实际旋转角        */
    Mat3 R  = rodrigues(wu, a);
    Mat3 imr = m3_add(m3_identity(), m3_scale(R, -1.0));   /* I - R */
    Vec3 t1 = m3_apply(imr, v3_cross(wu, v));              /* (I-R)(ω×v) */
    double h = v3_dot(wu, v);                              /* ω^T v     */
    Vec3 t2 = v3_scale(wu, h * a);                         /* ωω^T v θ  */
    g.R = R;
    g.p = v3_add(t1, t2);
    return g;
}

/* 伴随映射: Ad_g ξ = ( R v + p × (R ω),  R ω ) */
static Twist adjoint(SE3 g, Twist xi) {
    Twist y;
    y.w = m3_apply(g.R, xi.w);
    y.v = v3_add(m3_apply(g.R, xi.v), v3_cross(g.p, y.w));
    return y;
}

/* 旋转关节旋量: 轴方向 ω(单位向量), 轴上一点 q => v = -ω × q */
static Twist revolute_twist(Vec3 w, Vec3 q) {
    Twist t;
    t.w = w;
    t.v = v3_scale(v3_cross(w, q), -1.0);
    return t;
}

int main(void) {
    /* 绕穿过点 q=(0,1,0) 的 z 轴旋转 90 度 (MLS 例 2.3, l1 = 1) */
    Vec3 w = v3(0, 0, 1), q = v3(0, 1, 0);
    Twist xi = revolute_twist(w, q);
    SE3 g = exp_se3(xi.w, xi.v, PI / 2.0);
    printf("绕 z 轴(过 q=(0,1,0))旋转 90 度的齐次变换:\n");
    printf("  平移 p = (%.4f, %.4f, %.4f)\n", g.p.x, g.p.y, g.p.z);
    printf("  理论 p = (%.4f, %.4f, %.4f)   (l1 sinθ, l1(1-cosθ), 0)\n",
           1.0, 1.0, 0.0);

    Vec3 q0 = v3(1, 0, 0);                       /* 体坐标点 */
    Vec3 qa = v3_add(m3_apply(g.R, q0), g.p);    /* 变换后    */
    printf("  点 (1,0,0) -> (%.4f, %.4f, %.4f)  (期望 (1,2,0))\n",
           qa.x, qa.y, qa.z);

    /* 纯平移: ω = 0, v = (1,2,3), θ = 2 */
    SE3 gt = exp_se3(v3(0,0,0), v3(1,2,3), 2.0);
    printf("纯平移 θ=2: p = (%.4f, %.4f, %.4f)\n", gt.p.x, gt.p.y, gt.p.z);

    /* 伴随: 用 g 把体坐标旋量变换到空间坐标旋量 (MLS 2.57 式) */
    Twist xi_body = {v3(1,0,0), v3(0,0,1)};
    Twist xi_sp  = adjoint(g, xi_body);
    printf("Ad_g ξ: v = (%.4f, %.4f, %.4f), ω = (%.4f, %.4f, %.4f)\n",
           xi_sp.v.x, xi_sp.v.y, xi_sp.v.z,
           xi_sp.w.x, xi_sp.w.y, xi_sp.w.z);
    return 0;
}
```

## 5. 教材/论文精读

**MLS 第 2 章第 3 节：齐次表示、指数坐标与螺旋。** Murray、Li 与 Sastry 在《A Mathematical Introduction to Robotic Manipulation》第 2 章第 3 节用非常经济的方式完成了从旋转到完整刚性运动的推广。首先，通过给点追加分量 1、给向量追加分量 0，把仿射变换 $$q_a = R q_b + p$$ 线性化为 4x4 齐次矩阵，代价只是维度从 3 升到 4；末行零向量的"多余行李"在图形学里可被替换成透视或缩放变换，但那已不再是刚性运动。这节的精髓在于把 SE(3) 的四条群公理（封闭、单位元、逆、结合）全部归结为普通矩阵乘法的性质，从而让复合与求逆都变成机械运算。随后作者用单连杆的转动例子引出旋量 $$\xi = (-\omega\times q, \omega)$$，并证明 $$p(t) = e^{\hat{\xi}t}p(0)$$——常值旋量的指数恰好在单位速度下生成螺旋运动。这一"速度-位姿"的微分方程视角，是后续推导空间/体速度的伏笔。

**指数映射公式的推导与螺旋几何。** MLS 命题 2.8 通过分块上三角矩阵的技巧推导出闭合形式：先把旋量相似变换成只含对角块的 $$\hat{\xi}'$$，再利用矩阵指数的性质还原。结果即式 (2.36)：旋转分量用罗德里格斯公式，平移分量由 $$(I-e^{\hat{\omega}\theta})(\omega\times v) + \omega\omega^T v \theta$$ 给出。命题 2.9 进一步证明指数映射满射到 SE(3)：给定任意 g，先用 SO(3) 的指数求解 ω 与 θ，再解线性方程 (2.38) 反求 v；构造性证明同时给出了数值求旋量坐标的算法。第 3.3 节把旋量与螺旋（Chasles 定理）联系起来：螺距 $$h = \omega^T v/\lVert\omega\rVert^2$$、轴、模三个几何量给旋量以直观解释，也使旋转关节（零螺距）与移动关节（无穷螺距）的建模统一起来。需要留意的是指数映射**多对一**：同一 g 可由不同的 (ω, θ) 生成，2π 周期导致坐标歧义，工程上常约定 θ ∈ (0, 2π) 或 π 附近的折中。

**MLS 第 2 章第 4 节：刚体速度与伴随。** 这节处理一个常被初学者忽略的问题：SE(3) 不是向量空间，$$\dot{g}$$ 本身没有几何意义，必须借助 $$\dot{g}g^{-1}$$（空间速度）或 $$g^{-1}\dot{g}$$（体速度）把速度"装"进 se(3)。作者强调空间速度的线性分量并非体坐标原点的速度，而是"恰好穿过空间系原点瞬间的那个刚体点"的速度——这一反直觉的定义让许多人在仿真对拍时出错。伴随矩阵 $$\mathrm{Ad}_g$$ 统一了两类速度的换系，并推广为旋量轴的搬移规则 (2.64 式)，后者在第 3 章推导多连杆正运动学时成为核心工具。总体而言，MLS 第 2 章是全书几何语言的地基：它把 DH 参数化替换为坐标无关的旋量表示，代价是需要适应更抽象的李群/李代数记号——但对理解机构奇异性、可操作度等全局性质，这种抽象是值得的。另一个工程陷阱是旋转角 $$\theta$$ 的 2π 周期歧义：求解旋量坐标时必须约定取值范围并做好数值稳定性处理（例如对接近 0 或 2π 的角度使用级数截断）。

## 6. 实验/编程作业

对应 EECS C106A 第 2-3 周，建议按如下顺序练习：

1. **SE(3) 代数库**：用 Python 的 numpy（或本节 C 代码）实现 `se3_identity/mul/inv` 与 `transform_point`，数值验证群公理：对随机 (p, R) 检查 $$g g^{-1} = I_4$$、$$(g_1 g_2)^{-1} = g_2^{-1} g_1^{-1}$$。
2. **指数坐标双向换算**：实现罗德里格斯公式与 `exp_se3`，对绕任意轴（过任意点）的旋转构造旋量并求指数，与解析式对比；再实现逆过程（命题 2.9），验证 $$\exp \circ \log$$ 的往返误差。
3. **单连杆正运动学**：仿照 MLS 例 2.3，用指数公式计算一自由度机械臂末端位姿随 θ 变化的轨迹，并在三维绘图里画出螺旋运动。
4. **伴随与速度**：给定一段关节角轨迹 θ(t)，分别用数值微分计算 $$V^s$$ 与 $$V^b$$，验证 $$V^s = \mathrm{Ad}_g V^b$$，并对照"体速度恒定"的直觉理解空间速度为何不断变化。
5. **精度检查**：由于旋量指数涉及 sin/cos 与叉积，建议对第 1、2 步的往返误差（$$\exp(\log g)$$ 与 g 之差的范数）设置 1e-9 量级容差，并用 1e-6 的数值微分步长对拍解析结果，养成写数值测试的习惯。

## 7. 延伸阅读

- Murray, Li, Sastry. *A Mathematical Introduction to Robotic Manipulation*, CRC Press, 1994. 第 2 章（本章的主要依据），第 3 章（乘积指数正运动学）。
- Brockett, R. W. *Robotic manipulators and the product of exponentials formula*, in Mathematical Theory of Networks and Systems, 1984. 旋量指数用于运动学的原始文献。
- Lynch, K. M., Park, F. C. *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017. 第 3-4 章用更通俗的语言讲述 SE(3)、旋量与伴随，配套有开源 Python 库 `modern_robotics`。
- Selig, J. M. *Geometric Fundamentals of Robotics*, Springer, 2005. 从射影几何与旋量理论出发的进阶读物。
- 《机器人学导论》中文教材配套讲义与 EECS C106A 课程主页（第 2-3 周笔记与 lab 源码）。

## 8. 小结

本章把第 3 章的旋转运动推广为完整的刚性运动：SE(3) = R3 × SO(3) 的每个元素用一个 4x4 齐次矩阵同时编码位置与姿态，复合与求逆归结为矩阵乘法和转置。旋量 ξ = (v, ω) 作为 se(3) 的元素刻画刚体的瞬时运动，其指数映射 $$e^{\hat{\xi}\theta}$$ 统一了旋转与平移、并建立了与螺旋（轴、螺距、模）的几何对应。空间速度、体速度与伴随矩阵 $$\mathrm{Ad}_g$$ 提供了换系与搬移旋量轴的统一规则。这套工具消除了 DH 参数化对坐标系的依赖，为第 5 章以乘积指数公式为核心的正运动学打下基础。

---
[上一章](ch03_旋转运动与SO3.md) ｜ [下一章](ch05_正运动学.md)
