#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DH116S-R000-A1 灵巧手 MuJoCo 交互仿真
========================================
启动后通过窗口右侧 Actuators 面板拖动滑块控制各主动关节。
被动关节通过 equality 约束自动跟随主动关节。

运行:
    python sim_hand.py                # 仅仿真，不发送 UDP
    python sim_hand.py --udp          # 仿真 + UDP 发送角度
    python sim_hand.py --udp --port 12345  # 指定端口
"""

import argparse, os, sys, time, json, socket
import numpy as np

# Windows: 强制 UTF-8 输出，避免中文乱码
if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    sys.exit("[ERROR] 请先安装: pip install mujoco")

# ─── 路径 ───────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, ".", "DH116S-R000-A1.xml")

# ─── 关节描述（仅供终端打印） ────────────────────────────────────────
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


def print_info(model: "mujoco.MjModel") -> None:
    print("=" * 58)
    print("  DH116S-R000-A1  灵巧手  MuJoCo 交互仿真")
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
        print(f"    {aname:<16} [{np.degrees(lo):5.1f}° ~ {np.degrees(hi):5.1f}°]  {desc}")

    print()
    print("  ── 被动关节（equality 约束自动跟随）──")
    for pas, act, desc in PASSIVE_DESC:
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


# 需要发送的 6 个主动关节
ACTIVE_JOINTS = ["finger11", "finger12", "finger21", "finger31", "finger41", "finger51"]


def build_joint_info(model: "mujoco.MjModel") -> list:
    """构建需要发送的关节信息列表: [(name, jid, qposadr), ...]"""
    info = []
    for jname in ACTIVE_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        info.append((jname, jid, model.jnt_qposadr[jid]))
    return info


def send_joint_angles(sock: socket.socket, addr: tuple,
                      data: "mujoco.MjData", joint_info: list) -> None:
    """通过 UDP 发送 6 个主动关节的角度（度）"""
    angles = {}
    for jname, jid, qposadr in joint_info:
        angles[jname] = round(float(np.degrees(data.qpos[qposadr])), 2)
    msg = json.dumps(angles, ensure_ascii=False)
    sock.sendto(msg.encode("utf-8"), addr)


def run(model: "mujoco.MjModel", data: "mujoco.MjData",
        enable_udp: bool = False, udp_port: int = 9876) -> None:
    # ─── UDP 设置 ───
    UDP_IP = "127.0.0.1"
    sock = None
    joint_info = None
    prev_ctrl = None

    if enable_udp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        joint_info = build_joint_info(model)
        prev_ctrl = np.zeros(model.nu)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 初始相机视角：斜上方俯视，手掌正面朝向镜头
        viewer.cam.azimuth   = 140
        viewer.cam.elevation = -25
        viewer.cam.distance  = 0.35
        viewer.cam.lookat[:] = [0.0, 0.0, 0.07]

        if enable_udp:
            print(f"  [UDP] 关节角度将发送到 {UDP_IP}:{udp_port}")
        else:
            print("  [UDP] 未启用 (加 --udp 参数可开启)")
        print("  [仿真运行中] 请在窗口右侧 Actuators 面板拖动滑块来控制手指")
        print("  关闭窗口退出。\n")

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

            # 检测执行器控制值是否发生变化
            if enable_udp and not np.allclose(data.ctrl, prev_ctrl, atol=1e-6):
                send_joint_angles(sock, (UDP_IP, udp_port), data, joint_info)
                prev_ctrl[:] = data.ctrl

            # 和仿真 timestep 对齐，保持实时速率
            time.sleep(model.opt.timestep)

    if sock:
        sock.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="DH116S-R000-A1 MuJoCo 仿真")
    parser.add_argument("--udp", action="store_true",
                        help="启用 UDP 发送关节角度")
    parser.add_argument("--port", type=int, default=9876,
                        help="UDP 端口号 (默认 9876)")
    args = parser.parse_args()

    if not os.path.exists(MODEL_PATH):
        sys.exit(f"[ERROR] 找不到模型文件:\n  {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    print_info(model)
    run(model, data, enable_udp=args.udp, udp_port=args.port)


if __name__ == "__main__":
    main()
