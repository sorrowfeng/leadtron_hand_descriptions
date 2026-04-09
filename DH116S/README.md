# DH116S Joint README

## 概述

适用范围：

- 目录 `DH116S_URDF_MJCF_Files`
- 左手 `DH116S-L000-A1`
- 右手 `DH116S-R000-A1`

说明：

- 本手模型共有 6 个主动关节、5 个从动关节。
- 指尖纹路 link 为固定件，不是独立运动关节。
- 主从关系文件中的公式按原始文本保留，写法中的 `x2` 表示 `x^2`。

## 关节分类

### 主动关节

| 手指 | 关节名 | 描述 |
|---|---|---|
| 大拇指 | `finger11` | 大拇指侧摆主动关节 |
| 大拇指 | `finger12` | 大拇指弯曲主动关节 |
| 食指 | `finger21` | 食指弯曲主动关节 |
| 中指 | `finger31` | 中指弯曲主动关节 |
| 无名指 | `finger41` | 无名指弯曲主动关节 |
| 小拇指 | `finger51` | 小拇指弯曲主动关节 |

### 从动关节

| 手指 | 关节名 | 描述 |
|---|---|---|
| 大拇指 | `finger13` | 大拇指弯曲被动关节 |
| 食指 | `finger22` | 食指弯曲被动关节 |
| 中指 | `finger32` | 中指弯曲被动关节 |
| 无名指 | `finger42` | 无名指弯曲被动关节 |
| 小拇指 | `finger52` | 小拇指弯曲被动关节 |

### 固定 link

| Link 名 | 描述 |
|---|---|
| `base_link` | 基座 |
| `finger14_link` | 大拇指指尖纹路，固定在 `finger13_link` 上 |
| `finger23_link` | 食指指尖纹路，固定在 `finger22_link` 上 |
| `finger33_link` | 中指指尖纹路，固定在 `finger32_link` 上 |
| `finger43_link` | 无名指指尖纹路，固定在 `finger42_link` 上 |
| `finger53_link` | 小拇指指尖纹路，固定在 `finger52_link` 上 |

## 主从关系

下表整理主动关节与从动关节之间的关系。原始公式按源文件保留。

| 手指 | 主动关节 | 从动关节 | 关系公式 |
|---|---|---|---|
| 大拇指 | `finger12` | `finger13` | `y = -0.0008x2 + 0.929x - 0.8639` |
| 食指 | `finger21` | `finger22` | `y = 0.0067x2 + 0.8847x + 2.1926` |
| 中指 | `finger31` | `finger32` | `y = 0.0076x2 + 0.8485x + 2.0597` |
| 无名指 | `finger41` | `finger42` | `y = 0.0079x2 + 0.8321x + 2.4732` |
| 小拇指 | `finger51` | `finger52` | `y = 0.0075x2 + 0.7349x + 2.0181` |

补充说明：

- 大拇指另外还有一个独立主动侧摆关节 `finger11`。
- 其余四指各有 1 个主动弯曲关节和 1 个从动弯曲关节。
- 在当前 `URDF` 中，从动关节通过 `mimic` 表达主从依赖。
- 在当前 `MJCF` 中，从动关节通过 `equality joint polycoef` 表达更完整的联动关系。

## 各关节描述

| Link / Joint 名 | 类型 | 描述 |
|---|---|---|
| `base_link` | 固定基座 | 基座 |
| `finger11_link` / `finger11` | 主动关节 | 大拇指侧摆主动关节 |
| `finger12_link` / `finger12` | 主动关节 | 大拇指弯曲主动关节 |
| `finger13_link` / `finger13` | 从动关节 | 大拇指弯曲被动关节 |
| `finger14_link` | 固定 link | 大拇指指尖纹路，固定在 `finger13_link` 上 |
| `finger21_link` / `finger21` | 主动关节 | 食指弯曲主动关节 |
| `finger22_link` / `finger22` | 从动关节 | 食指弯曲被动关节 |
| `finger23_link` | 固定 link | 食指指尖纹路，固定在 `finger22_link` 上 |
| `finger31_link` / `finger31` | 主动关节 | 中指弯曲主动关节 |
| `finger32_link` / `finger32` | 从动关节 | 中指弯曲被动关节 |
| `finger33_link` | 固定 link | 中指指尖纹路，固定在 `finger32_link` 上 |
| `finger41_link` / `finger41` | 主动关节 | 无名指弯曲主动关节 |
| `finger42_link` / `finger42` | 从动关节 | 无名指弯曲被动关节 |
| `finger43_link` | 固定 link | 无名指指尖纹路，固定在 `finger42_link` 上 |
| `finger51_link` / `finger51` | 主动关节 | 小拇指弯曲主动关节 |
| `finger52_link` / `finger52` | 从动关节 | 小拇指弯曲被动关节 |
| `finger53_link` | 固定 link | 小拇指指尖纹路，固定在 `finger52_link` 上 |

## 各关节最大行程

| 关节说明 | 关节名 | 最大角度 (deg) | 最大行程 (rad) |
|---|---|---:|---:|
| 大拇指侧摆 | `finger11` | 100 | 1.75 |
| 大拇指弯曲1 | `finger12` | 80 | 1.40 |
| 大拇指弯曲2 | `finger13` | 65 | 1.13 |
| 食指弯曲1 | `finger21` | 80 | 1.40 |
| 食指弯曲2 | `finger22` | 100 | 1.75 |
| 中指弯曲1 | `finger31` | 80 | 1.40 |
| 中指弯曲2 | `finger32` | 100 | 1.75 |
| 无名指弯曲1 | `finger41` | 80 | 1.40 |
| 无名指弯曲2 | `finger42` | 100 | 1.75 |
| 小拇指弯曲1 | `finger51` | 80 | 1.40 |
| 小拇指弯曲2 | `finger52` | 100 | 1.75 |

## 汇总视图

| 手指 | 主动关节 | 从动关节 | 固定指尖 link | 最大行程 |
|---|---|---|---|---|
| 大拇指 | `finger11`, `finger12` | `finger13` | `finger14_link` | `finger11=1.75`, `finger12=1.40`, `finger13=1.13` |
| 食指 | `finger21` | `finger22` | `finger23_link` | `finger21=1.40`, `finger22=1.75` |
| 中指 | `finger31` | `finger32` | `finger33_link` | `finger31=1.40`, `finger32=1.75` |
| 无名指 | `finger41` | `finger42` | `finger43_link` | `finger41=1.40`, `finger42=1.75` |
| 小拇指 | `finger51` | `finger52` | `finger53_link` | `finger51=1.40`, `finger52=1.75` |
