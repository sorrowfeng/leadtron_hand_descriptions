#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "mujoco>=3.1",
#     "numpy",
# ]
# ///
"""
DH116-R000-A1 灵巧手 MuJoCo 交互仿真
========================================
启动后通过窗口右侧 Actuators 面板拖动滑块控制各主动关节。
被动关节通过 equality 约束自动跟随主动关节。

运行:
    uv run sim_hand.py

  首次运行 uv 会依据文件头部的 PEP 723 块自动建环境、下载 mujoco / numpy，
  之后有缓存，秒开。

macOS 说明:
    交互窗口必须由 mjpython 启动（GUI 调用需在主线程）。脚本会自动检测并
    重启自己，所以直接 `uv run sim_hand.py` 即可。等价的手动写法：
        uv run --with mujoco --with numpy -- mjpython sim_hand.py
"""

import glob, os, sys, time
import numpy as np

if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    sys.exit("[ERROR] 请先安装: pip install mujoco")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = ""


def resolve_model_path() -> str:
    """定位 MJCF：命令行参数 > 环境变量 DH116_MODEL > 同目录下唯一的 .xml。"""
    if len(sys.argv) > 1 and sys.argv[1].lower().endswith(".xml"):
        return os.path.abspath(sys.argv[1])
    env = os.environ.get("DH116_MODEL")
    if env:
        return os.path.abspath(env)
    xmls = sorted(glob.glob(os.path.join(SCRIPT_DIR, "*.xml")))
    if len(xmls) == 1:
        return xmls[0]
    hint = ("候选: " + ", ".join(os.path.basename(x) for x in xmls)) if xmls else "目录内没有 .xml"
    raise SystemExit("[ERROR] 无法确定模型文件\n"
                     "        用法: uv run sim_hand.py [模型.xml]\n"
                     f"        {hint}")

ACTIVE_DESC = {
    "act_finger11": "拇指侧摆  (MCP ab/adduction)",
    "act_finger12": "拇指弯曲  (MCP flexion)",
    "act_finger21": "食指弯曲  (MCP flexion)",
    "act_finger31": "中指弯曲  (MCP flexion)",
    "act_finger41": "无名指弯曲(MCP flexion)",
    "act_finger51": "小拇指弯曲(MCP flexion)",
}
PASSIVE_DESC = [
    ("finger13", "finger12", "拇指 IP   ← 拇指 MCP"),
    ("finger22", "finger21", "食指 PIP  ← 食指 MCP"),
    ("finger32", "finger31", "中指 PIP  ← 中指 MCP"),
    ("finger42", "finger41", "无名 PIP  ← 无名 MCP"),
    ("finger52", "finger51", "小指 PIP  ← 小指 MCP"),
]



def ensure_mjpython() -> None:
    """macOS: MuJoCo 的交互窗口必须由 mjpython 启动（GUI 调用需在主线程）。"""
    if sys.platform != "darwin" or os.environ.get("_DH116_MJPYTHON"):
        return
    try:
        from mujoco import viewer as _viewer
    except ImportError:
        return
    if getattr(_viewer, "_MJPYTHON", None) is not None:
        return  # already running under mjpython
    launcher = os.path.join(os.path.dirname(sys.executable), "mjpython")
    if not os.path.exists(launcher):
        return  # let mujoco raise its own, more informative error
    os.environ["_DH116_MJPYTHON"] = "1"
    print("  [macOS] 交互窗口需要用 mjpython 启动，正在自动切换 ...")
    try:
        os.execv(launcher, [launcher, os.path.abspath(__file__), *sys.argv[1:]])
    except OSError as exc:  # pragma: no cover
        sys.exit(f"[ERROR] 无法启动 mjpython: {exc}\n请手动运行: mjpython {__file__}")

def print_info(model: "mujoco.MjModel") -> None:
    print("=" * 58)
    print(f"  {os.path.splitext(os.path.basename(MODEL_PATH))[0]}  灵巧手  MuJoCo 交互仿真")
    print("=" * 58)
    print(f"  MuJoCo {mujoco.__version__}")
    print(f"  关节: {model.njnt}  执行器: {model.nu}  equality: {model.neq}")
    print()

    print("  ── 主动关节（右侧 Actuators 面板拖动滑块控制）──")
    for i in range(model.nu):
        aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        jid   = model.actuator_trnid[i, 0]
        lo, hi = model.jnt_range[jid]
        desc  = ACTIVE_DESC.get(aname, "")
        print(f"    {aname:<16} [{np.degrees(lo):6.1f}° ~ {np.degrees(hi):6.1f}°]  {desc}")

    print()
    print("  ── 被动关节（equality 约束自动跟随）──")
    for _, _, desc in PASSIVE_DESC:
        print(f"    {desc}")

    print()
    print("  ── 操作提示 ──")
    print("    Actuators 面板滑块  → 控制各手指弯曲/侧摆角度")
    print("    鼠标左键拖动        → 旋转视角")
    print("    鼠标右键拖动        → 平移视角")
    print("    滚轮                → 缩放")
    print("    Space               → 暂停 / 继续仿真")
    print("    Backspace           → 重置到初始状态")
    print("    关闭窗口            → 退出程序")
    print()


def run(model: "mujoco.MjModel", data: "mujoco.MjData") -> None:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth   = 140
        viewer.cam.elevation = -25
        viewer.cam.distance  = 0.35
        viewer.cam.lookat[:] = [0.0, 0.0, 0.07]

        print("  [仿真运行中] 请在窗口右侧 Actuators 面板拖动滑块来控制手指")
        print("  关闭窗口退出。")

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


def main() -> None:
    global MODEL_PATH
    ensure_mjpython()
    MODEL_PATH = resolve_model_path()
    if not os.path.exists(MODEL_PATH):
        sys.exit(f"[ERROR] 找不到模型文件: {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    print_info(model)
    run(model, data)


if __name__ == "__main__":
    main()
