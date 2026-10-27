# DH116 Joint README

## 概述

适用范围：

- 目录 `DH116_URDF_MJCF_Files`
- 左手 `DH116-L000-A1`
- 右手 `DH116-R000-A1`

说明：

- 本手模型共有 6 个主动关节、5 个从动关节。
- 指尖纹路 link 为固定件，不是独立运动关节。
- 主从关系公式按原始文本保留，写法中的 `x2` 表示 `x^2`。
- 当前 `URDF` 通过 `mimic` 表达线性主从关系（行程区间上的弦线拟合，两端与二次曲线重合）；当前 `MJCF` 通过 `equality joint polycoef` 表达完整的二次联动关系。
- `DH116` 左右手模型当前都统一使用小写 `link` 命名。
- 左右手共用同一套行程与联动参数，由 `tools/patch_sw2urdf_limits.py` 统一写入。

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

| 手指 | 右手固定 link | 左手固定 link | 描述 |
|---|---|---|---|
| 大拇指 | `finger14_link` | `finger14_link` | 固定在 `finger13_*` 上的指尖纹路 |
| 食指 | `finger23_link` | `finger23_link` | 固定在 `finger22_*` 上的指尖纹路 |
| 中指 | `finger33_link` | `finger33_link` | 固定在 `finger32_*` 上的指尖纹路 |
| 无名指 | `finger43_link` | `finger43_link` | 固定在 `finger42_*` 上的指尖纹路 |
| 小拇指 | `finger53_link` | `finger53_link` | 固定在 `finger52_*` 上的指尖纹路 |

## 主从关系

下表整理主动关节与从动关节之间的关系。原始公式按源文件保留。

| 手指 | 主动关节 | 从动关节 | 原始公式 |
|---|---|---|---|
| 大拇指 | `finger12` | `finger13` | `y = 0.0082x2 + 0.8808x + 0.2631` |
| 食指 | `finger21` | `finger22` | `y = 0.0035x2 + 0.9206x + 0.6344` |
| 中指 | `finger31` | `finger32` | `y = 0.0035x2 + 0.9206x + 0.6344` |
| 无名指 | `finger41` | `finger42` | `y = 0.0035x2 + 0.9206x + 0.6344` |
| 小拇指 | `finger51` | `finger52` | `y = 0.0035x2 + 0.9206x + 0.6344` |

### URDF mimic 线性近似

URDF 的 `<mimic>` 只支持线性关系，二次项无法表达，因此用**行程区间上的弦线**代替二次曲线：
过 `(0, c0)` 与 `(行程, y_行程)` 两点，两端完全重合，中段最大偏差 `c2·行程²/4`（3.4~3.8°）。
直接取一阶项（`multiplier = c1`）会让从动关节在满行程处少转 14~15°。

| 从动关节 | 主动关节 | multiplier | offset (rad) | 中段最大偏差 |
|---|---|---:|---:|---:|
| `finger13` | `finger12` | 1.2334 | 0.004592 | 3.79° |
| `finger22` | `finger21` | 1.1411 | 0.011072 | 3.47° |
| `finger32` | `finger31` | 1.1411 | 0.011072 | 3.47° |
| `finger42` | `finger41` | 1.1376 | 0.011072 | 3.36° |
| `finger52` | `finger51` | 1.1516 | 0.011072 | 3.81° |

### MJCF equality polycoef

换算规则：

- `y_deg = C0 + C1*x_deg + C2*x_deg^2`
- `c0 = C0 * (π / 180)`
- `c1 = C1`
- `c2 = C2 * (180 / π)`
- `polycoef = "c0 c1 c2 0 0"`

| 从动关节 | 主动关节 | polycoef |
|---|---|---|
| `finger13` | `finger12` | `0.004592 0.8808 0.469825 0 0` |
| `finger22` | `finger21` | `0.011072 0.9206 0.200535 0 0` |
| `finger32` | `finger31` | `0.011072 0.9206 0.200535 0 0` |
| `finger42` | `finger41` | `0.011072 0.9206 0.200535 0 0` |
| `finger52` | `finger51` | `0.011072 0.9206 0.200535 0 0` |

## 各关节范围（当前 DH116 模型，左右手相同）

| 关节名 | 类型 | 范围 (rad) | 范围 (deg) |
|---|---|---|---|
| `finger11` | 主动 | `[0, 0.994838]` | `[0, 57]` |
| `finger12` | 主动 | `[0, 0.750492]` | `[0, 43]` |
| `finger13` | 从动 | `[0.004592, 0.930248]` | `[0.2631, 53.2993]` |
| `finger21` | 主动 | `[0, 1.099557]` | `[0, 63]` |
| `finger22` | 从动 | `[0.011072, 1.265777]` | `[0.6344, 72.5237]` |
| `finger31` | 主动 | `[0, 1.099557]` | `[0, 63]` |
| `finger32` | 从动 | `[0.011072, 1.265777]` | `[0.6344, 72.5237]` |
| `finger41` | 主动 | `[0, 1.082104]` | `[0, 62]` |
| `finger42` | 从动 | `[0.011072, 1.242074]` | `[0.6344, 71.1656]` |
| `finger51` | 主动 | `[0, 1.151917]` | `[0, 66]` |
| `finger52` | 从动 | `[0.011072, 1.337620]` | `[0.6344, 76.6400]` |

从动关节的下界是公式的常数项，上界是二次公式在主动关节上界处的取值（包住弦线的线性输出）。

## 文件结构

新增 / 维护的相关文件如下：

- `DH116_URDF_MJCF_Files/DH116-R000-A1/urdf/DH116-R000-A1.urdf`
- `DH116_URDF_MJCF_Files/DH116-L000-A1/urdf/DH116-L000-A1.urdf`
- `DH116_URDF_MJCF_Files/DH116-R000-A1/mjcf/DH116-R000-A1.xml`
- `DH116_URDF_MJCF_Files/DH116-L000-A1/mjcf/DH116-L000-A1.xml`
- `DH116_URDF_MJCF_Files/DH116-R000-A1/mjcf/sim_hand.py`
- `DH116_URDF_MJCF_Files/DH116-L000-A1/mjcf/sim_hand.py`

### 从 SolidWorks 重新导出后的处理

SolidWorks 导出会覆盖 `urdf/`、`meshes/`，并把 `mimic`、标定限位、胶垫颜色、链接名大小写全部还原成导出值，因此每次导出后跑：

```bash
python3 tools/patch_sw2urdf_limits.py -p DH116/DH116_URDF_MJCF_Files/DH116-R000-A1 --tidy
python3 tools/urdf2mjcf.py             -p DH116/DH116_URDF_MJCF_Files/DH116-R000-A1
python3 tools/patch_sw2urdf_limits.py -p DH116/DH116_URDF_MJCF_Files/DH116-L000-A1 --tidy
python3 tools/urdf2mjcf.py             -p DH116/DH116_URDF_MJCF_Files/DH116-L000-A1
```

`--tidy` 同时会把文本文件转 LF、删除 `export.log`/`.DS_Store`/空 `textures/`，并把 `*_Link` 的链接名与网格文件名规范成 `*_link`。

### 运行 MuJoCo 仿真

```bash
cd DH116_URDF_MJCF_Files/DH116-R000-A1/mjcf   # 或 L000
uv run sim_hand.py        # 首次自动装 mujoco/numpy；macOS 自动切到 mjpython
```

## 汇总视图

| 手指 | 主动关节 | 从动关节 | 固定指尖 link |
|---|---|---|---|
| 大拇指 | `finger11`, `finger12` | `finger13` | `finger14_link` / `finger14_link` |
| 食指 | `finger21` | `finger22` | `finger23_link` / `finger23_link` |
| 中指 | `finger31` | `finger32` | `finger33_link` / `finger33_link` |
| 无名指 | `finger41` | `finger42` | `finger43_link` / `finger43_link` |
| 小拇指 | `finger51` | `finger52` | `finger53_link` / `finger53_link` |
