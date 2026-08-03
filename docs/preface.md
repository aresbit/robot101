---
layout: default
title: 前言与导读
---

# 前言与导读

## 这本书的来历

加州大学伯克利分校的 EECS C106A/206A《Introduction to Robotics》是机器人学导论的世界级课程。
Fall 2023 由 Koushil Sreenath 与 Shankar Sastry 两位教授讲授，采用 Murray/Li/Sastry《A Mathematical Introduction to Robotic Manipulation》(MLS) 为官方教材，
以严谨的**旋量理论 (screw theory)** 贯穿全书——这与大多数 DH 参数化的教材风格不同，更加统一、优雅。

本书把该课程的教学日历（14 周：刚性运动 → 运动学 → 视觉 → 速度/雅可比 → 动力学 → 控制 → 规划 → 前沿）编译为一本中文教材，
并加入 C 语言实现示例。**所有讲义与教材版权归原作者与加州大学伯克利分校所有，本书仅做学术整理。**

## 谁应该读

- 想系统进入机器人领域的学生 / 工程师。
- 已经会用 ROS/PyBullet 调包，但想理解旋转矩阵、四元数、雅可比背后的数学。
- 需要把机器人算法（PID、RRT、IK）用 C 语言落地的嵌入式 / 系统工程师。

## 怎么读

按顺序读最稳。第 3-4 章（SO(3)/SE(3) 与旋量）是全书的数学地基，值得精读。
第 5-12 章是运动学—视觉—动力学—控制—规划主线。第 13-14 章是应用与展望。

每章配 C 实现示例。**强烈建议把 C 代码敲一遍、编译通过、跑几个例子**——机器人学的数学如果只看公式很容易"觉得自己懂了"，
而写代码会强迫你把每一步矩阵运算、每一次坐标变换落到实处。

## 致谢

- Koushil Sreenath 与 Shankar Sastry 教授把 EECS C106A 的课程资料公开在 GitHub Pages 上。
- Richard Murray、Zexiang Li、Shankar Sastry 的 MLS 教材是本书的数学骨架，其 PDF 由课程网站公开提供。
- 加州大学伯克利分校 106A 教学团队的助教们把实验（ROS/机械臂/TurtleBot）设计得如此扎实。

## 版权

本书中文编译部分（章节行文与 C 代码示例）按 CC BY-NC-SA 4.0 发布。
课程讲义、MLS 教材、论文的版权归属原作者与学校；本书仅在合理使用范围内引用与摘录。

—— 编译者，2026 年