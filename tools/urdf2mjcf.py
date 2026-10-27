#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the MuJoCo MJCF model from the (patched) SW2URDF URDF.

The MJCF mirrors what the URDF carries: body tree, joint origins/axes/ranges,
inertial properties, visual colours.  The only values that do not exist in the
URDF are the quadratic coupling terms, which come from
``patch_sw2urdf_limits.CONFIG["coupling"]`` and are converted to MuJoCo
``polycoef`` with the README rule:

    y_deg = c0 + c1*x_deg + c2*x_deg^2      (angles in degrees)
    polycoef = [c0*pi/180, c1, c2*180/pi, 0, 0]      (angles in radians)

Usage
-----
    # URDF is assumed to be patched already (run patch_sw2urdf_limits.py first)
    python3 tools/patch_sw2urdf_limits.py -p <pkg>
    python3 tools/urdf2mjcf.py -p <pkg>                 # -> <pkg>/mjcf/<robot>.xml

    python3 tools/urdf2mjcf.py -p <pkg> --dry-run       # print only
    python3 tools/urdf2mjcf.py -p <pkg> --check         # exit 1 if out of date
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from patch_sw2urdf_limits import CONFIG as URDF_CONFIG  # noqa: E402

DEG = math.pi / 180.0

# --------------------------------------------------------------------------- #
# MJCF-specific settings (mirrors the hand-written model of the previous hand)
# --------------------------------------------------------------------------- #

SETTINGS = {
    "compiler": {"angle": "radian", "eulerseq": "XYZ", "autolimits": "true"},
    "option": {"gravity": "0 0 -9.81", "timestep": "0.002", "iterations": "100"},
    "defaults": {
        "joint": {"type": "hinge", "damping": "2.0", "armature": "0.01", "frictionloss": "0"},
        "geom": {"condim": "4", "friction": "1.0 0.005 0.001", "contype": "0", "conaffinity": "0"},
        "equality": {"solref": "0.004 1", "solimp": "0.9 0.99 0.001 0.5 2"},
    },
    # links that take part in collision; the palm and the first phalanx of each
    # finger stay non-colliding (same choice as the previous hand model)
    "collide": [
        "finger13_link", "finger14_link",
        "finger22_link", "finger23_link",
        "finger32_link", "finger33_link",
        "finger42_link", "finger43_link",
        "finger52_link", "finger53_link",
    ],
    "actuator": {"kp": "50", "kv": "2.0"},
    # section headers, keyed by the digit in the link name
    "finger_titles": {
        "1": "手指 1：大拇指 (Thumb)",
        "2": "手指 2：食指 (Index)",
        "3": "手指 3：中指 (Middle)",
        "4": "手指 4：无名指 (Ring)",
        "5": "手指 5：小拇指 (Pinky)",
    },
    "equality_names": {
        "finger13": "eq_thumb",
        "finger22": "eq_index",
        "finger32": "eq_middle",
        "finger42": "eq_ring",
        "finger52": "eq_pinky",
    },
    "meshdir": "../meshes",
}


# --------------------------------------------------------------------------- #
# URDF parsing
# --------------------------------------------------------------------------- #

def num(value: float | str, sig: int = 9) -> str:
    """Compact number formatting (0 -> '0', 1.0 -> '1', keeps exponents)."""
    x = float(value)
    if x == 0:
        return "0"
    s = f"{x:.{sig}g}"
    return s.replace("e", "E")


def vec(values) -> str:
    return " ".join(num(v) for v in values)


def xyz_attr(el, default="0 0 0"):
    return [float(v) for v in (el.get("xyz", default) if el is not None else default).split()]


def parse_urdf(path: Path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for link in root.findall("link"):
        name = link.get("name")
        inert = link.find("inertial")
        vis = link.find("visual")
        mesh = None
        rgba = "1 1 1 1"
        if vis is not None:
            m = vis.find("geometry/mesh")
            if m is not None:
                mesh = Path(m.get("filename").split("://")[-1]).name
            col = vis.find("material/color")
            if col is not None:
                rgba = col.get("rgba")
        links[name] = {
            "name": name,
            "mesh": mesh,
            "rgba": rgba,
            "mass": float(inert.find("mass").get("value")) if inert is not None else 0.0,
            "com": xyz_attr(inert.find("origin") if inert is not None else None),
            "inertia": {k: float(inert.find("inertia").get(k)) for k in
                        ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
            if inert is not None else {k: 0.0 for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")},
        }
    for j in root.findall("joint"):
        origin = j.find("origin")
        axis = j.find("axis")
        limit = j.find("limit")
        mimic = j.find("mimic")
        joints[j.get("name")] = {
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": xyz_attr(origin),
            "rpy": [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()],
            "axis": xyz_attr(axis) if axis is not None else [0, 0, 0],
            "range": [float(limit.get("lower", 0)), float(limit.get("upper", 0))] if limit is not None else None,
            "mimic": (mimic.get("joint"), float(mimic.get("multiplier", 1)),
                      float(mimic.get("offset", 0))) if mimic is not None else None,
        }
    return links, joints


def build_tree(links: dict, joints: dict, root: str = "base_link"):
    children: dict = {name: [] for name in links}
    for j in joints.values():
        children[j["parent"]].append(j["child"])
    return children


# --------------------------------------------------------------------------- #
# MJCF emission
# --------------------------------------------------------------------------- #

def inertia_attr(inertia: dict) -> str:
    # MuJoCo fullinertia order: ixx iyy izz ixy ixz iyz
    return " ".join(num(inertia[k], 7) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))


def geom_line(indent: str, name: str, mesh: str, rgba: str, collide: bool) -> str:
    extra = ' contype="1" conaffinity="1"' if collide else ""
    return f'{indent}<geom name="{name}" type="mesh" mesh="{mesh}" rgba="{rgba}"{extra}/>'


def inertial_lines(indent: str, link: dict) -> list[str]:
    pad = " " * (len(indent) + len("<inertial "))
    return [
        f'{indent}<inertial pos="{vec(link["com"])}"',
        f'{pad}mass="{num(link["mass"])}"',
        f'{pad}fullinertia="{inertia_attr(link["inertia"])}"/>',
    ]


def body_open(indent: str, name: str, pos, rpy) -> str:
    attrs = f'pos="{vec(pos)}" euler="{vec(rpy)}"'
    one_line = f'{indent}<body name="{name}" {attrs}>'
    if len(one_line) <= 88 or rpy == [0, 0, 0] and len(one_line) <= 96:
        return one_line
    pad = " " * (len(indent) + len("<body "))
    return f'{indent}<body name="{name}"\n{pad}{attrs}>'


def emit_mjcf(links: dict, joints: dict, robot: str, coupling: dict) -> str:
    children = build_tree(links, joints)
    title_map = SETTINGS["finger_titles"]
    collide = set(SETTINGS["collide"])
    out: list[str] = []

    out.append('<?xml version="1.0" encoding="utf-8"?>')
    out.append("<!--")
    out.append(f"  MuJoCo MJCF model for {robot} Dexterous Hand (Right)")
    out.append("  ============================================================")
    out.append("  Generated by tools/urdf2mjcf.py from urdf/" + robot + ".urdf")
    out.append("  Do not edit by hand: re-run the generator instead.")
    out.append("-->")
    out.append("")
    out.append(f'<mujoco model="{robot}">')
    out.append("")

    c = SETTINGS["compiler"]
    out.append(f'  <compiler angle="{c["angle"]}" meshdir="{SETTINGS["meshdir"]}" '
               f'eulerseq="{c["eulerseq"]}" autolimits="{c["autolimits"]}"/>')
    o = SETTINGS["option"]
    out.append(f'  <option gravity="{o["gravity"]}" timestep="{o["timestep"]}" '
               f'iterations="{o["iterations"]}"/>')
    out.append("")
    d = SETTINGS["defaults"]
    out.append("  <default>")
    out.append('    <joint ' + " ".join(f'{k}="{v}"' for k, v in d["joint"].items()) + "/>")
    out.append('    <geom ' + " ".join(f'{k}="{v}"' for k, v in d["geom"].items()) + "/>")
    out.append('    <equality ' + " ".join(f'{k}="{v}"' for k, v in d["equality"].items()) + "/>")
    out.append("  </default>")
    out.append("")
    out.append("  <asset>")
    for name, link in links.items():
        if link["mesh"]:
            out.append(f'    <mesh name="{name}" file="{link["mesh"]}"/>')
    out.append("  </asset>")
    out.append("")
    out.append("  <worldbody>")
    out.append("")

    def walk(link_name: str, indent: str, is_root: bool, last_finger: str | None):
        link = links[link_name]
        nonlocal state
        if is_root:
            out.append(f'{indent}<body name="{link_name}" pos="0 0 0">')
        else:
            joint = state["joint_of"][link_name]
            out.append(body_open(indent, link_name, joint["xyz"], joint["rpy"]))
            if joint["type"] != "fixed":
                lo, hi = joint["range"]
                out.append(f'{indent}  <joint name="{joint["name"]}" '
                           f'axis="{vec(joint["axis"])}" range="{num(lo)} {num(hi)}"/>')
        inner = indent + "  " if is_root else indent + "  "
        out.append(geom_line(inner, link_name, link_name, link["rgba"],
                             link_name in collide))
        out.extend(inertial_lines(inner, link))
        for child in children[link_name]:
            m = re.match(r"finger(\d)", child)
            digit = m.group(1) if m else ""
            if digit and digit != last_finger and digit in title_map:
                out.append("")
                out.append(f"{inner}<!-- ===== {title_map[digit]} ===== -->")
                last_finger = digit
            walk(child, inner, False, last_finger)
        out.append(f"{indent}</body>")

    state = {"joint_of": {j["child"]: j for j in joints.values()}}
    walk("base_link", "    ", True, None)

    out.append("")
    out.append("  </worldbody>")
    out.append("")
    out.append("  <equality>")
    for joint_name, j in joints.items():
        if not j["mimic"]:
            continue
        driver, mult, offset = j["mimic"]
        spec = coupling.get(joint_name)
        if spec is None:
            raise SystemExit(f"{joint_name}: no coupling entry (c2) in CONFIG")
        if spec["driver"] != driver:
            raise SystemExit(
                f"{joint_name}: URDF mimic is driven by {driver!r}, "
                f"CONFIG says {spec['driver']!r}"
            )
        # The quadratic lives in CONFIG; the URDF <mimic> is only used for the
        # pairing and is deliberately linearised (see patch_sw2urdf_limits.py).
        c0 = spec["c0"] * DEG
        c1 = spec["c1"]
        c2 = spec["c2"] * 180.0 / math.pi
        poly = f"{num(c0)} {num(c1)} {num(c2)} 0 0"
        eq_name = SETTINGS["equality_names"].get(joint_name, f"eq_{joint_name}")
        out.append(f'    <joint name="{eq_name}" joint1="{joint_name}" joint2="{driver}"')
        out.append(f'           polycoef="{poly}"/>')
    out.append("  </equality>")
    out.append("")
    out.append("  <actuator>")
    for joint_name, j in joints.items():
        if j["mimic"] or j["type"] == "fixed":
            continue
        lo, hi = j["range"]
        out.append(f'    <position name="act_{joint_name}" joint="{joint_name}" '
                   f'kp="{SETTINGS["actuator"]["kp"]}" kv="{SETTINGS["actuator"]["kv"]}" '
                   f'ctrlrange="{num(lo)} {num(hi)}"/>')
    out.append("  </actuator>")
    out.append("")
    out.append("</mujoco>")
    out.append("")
    return "\n".join(out)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def find_urdf(pkg: Path, explicit: Path | None) -> Path:
    if explicit:
        return explicit
    found = sorted((pkg / "urdf").glob("*.urdf"))
    if len(found) != 1:
        raise SystemExit(f"expected exactly one .urdf in {pkg / 'urdf'}, found {len(found)}")
    return found[0]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--package", type=Path, required=True)
    ap.add_argument("--urdf", type=Path)
    ap.add_argument("-o", "--output", type=Path, help="output .xml (default <pkg>/mjcf/<robot>.xml)")
    ap.add_argument("--dry-run", action="store_true", help="print the model, write nothing")
    ap.add_argument("--check", action="store_true", help="exit 1 if the file is out of date")
    ap.add_argument("-q", "--quiet", action="store_true")
    args = ap.parse_args(argv)

    urdf_path = find_urdf(args.package, args.urdf)
    links, joints = parse_urdf(urdf_path)
    robot = ET.parse(urdf_path).getroot().get("name")

    root_links = [n for n in links if n not in {j["child"] for j in joints.values()}]
    if root_links != ["base_link"]:
        raise SystemExit(f"unexpected root link(s): {root_links}")

    xml = emit_mjcf(links, joints, robot, URDF_CONFIG["coupling"])

    out_path = args.output or (args.package / "mjcf" / f"{robot}.xml")
    old = out_path.read_text(encoding="utf-8") if out_path.exists() else None
    changed = old != xml

    if args.dry_run:
        print(xml)
        return 0
    if args.check:
        if changed:
            print(f"OUT OF DATE: {out_path}", file=sys.stderr)
            return 1
        print("in sync")
        return 0

    if changed:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(xml, encoding="utf-8")
        print(f"wrote {out_path}")
    else:
        print(f"unchanged {out_path}")

    if not args.quiet:
        print(f"bodies: {len(links)}  joints: {len(joints)}  "
              f"equality: {sum(1 for j in joints.values() if j['mimic'])}  "
              f"actuators: {sum(1 for j in joints.values() if j['type'] != 'fixed' and not j['mimic'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
