#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-apply the DH116 mimic couplings + joint travel limits to a SW2URDF export.

SolidWorks -> URDF Exporter overwrites urdf/*.urdf on every export, which wipes
the hand-maintained <mimic> tags and the calibrated joint limits.  Run this
script after each export to put them back.

What it does
------------
URDF (urdf/<name>.urdf):
  * sets <limit lower/upper> on the active joints from the travel table below
  * sets <limit lower/upper> on the passive (follower) joints:
        lower = constant term of the coupling formula
        upper = full quadratic formula evaluated at the driver upper limit
  * inserts / rewrites the <mimic> tag of every follower joint
  * zeroes the <axis> of fixed joints (SolidWorks writes the normal of the
    reference geometry there, which is meaningless for a rigid connection)
  * keeps a documentation comment between the @@AUTO-LIMITS@@ markers
  * pins the visual rgba of the fingertip pads (CONFIG["colors"]) so all five
    silicone pads keep the same black
CSV (urdf/<name>.csv):
  * syncs the "Limit Lower" / "Limit Upper" columns and the four colour columns
    so a later "Import CSV -> Export URDF" keeps the same numbers and colours
Everything else (geometry, inertials, origins, axes, formatting) is left
byte-for-byte untouched.

With --tidy the whole package is normalised as well: every text file gets LF
line endings, export.log / .DS_Store are deleted and an empty textures/ folder
is removed - the SolidWorks exporter re-creates all of them on every run.

Usage
-----
    # patch URDF + CSV of a package folder
    python3 tools/patch_sw2urdf_limits.py -p DH116/DH116_URDF_MJCF_Files/DH116-R000-A1 --tidy

    # preview only / CI check
    python3 tools/patch_sw2urdf_limits.py -p <pkg> --dry-run
    python3 tools/patch_sw2urdf_limits.py -p <pkg> --check   # exit 1 if out of sync

    # custom travel table (JSON), e.g. for another hand variant
    python3 tools/patch_sw2urdf_limits.py -p <pkg> --config my_hand.json

The MJCF is generated from the patched URDF with tools/urdf2mjcf.py:
    python3 tools/urdf2mjcf.py -p <pkg>

The script is idempotent: running it twice changes nothing the second time.
"""

from __future__ import annotations

import argparse
import csv
import os
import io
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

DEG = math.pi / 180.0

# --------------------------------------------------------------------------- #
# Configuration (override any key with --config <file.json>)
# --------------------------------------------------------------------------- #

CONFIG = {
    "robot": "DH116-R000-A1",

    # travel of the six independent joints, degrees, measured from q = 0
    "travel_deg": {
        "finger11": 57.0,
        "finger12": 43.0,
        "finger21": 63.0,
        "finger31": 63.0,
        "finger41": 62.0,
        "finger51": 66.0,
    },

    # passive joint <- driver, coupling formula y = c2*x^2 + c1*x + c0 (degrees)
    # y = follower angle [deg], x = driver angle [deg]
    "coupling": {
        "finger13": {"driver": "finger12", "c2": 0.0082, "c1": 0.8808, "c0": 0.2631},
        "finger22": {"driver": "finger21", "c2": 0.0035, "c1": 0.9206, "c0": 0.6344},
        "finger32": {"driver": "finger31", "c2": 0.0035, "c1": 0.9206, "c0": 0.6344},
        "finger42": {"driver": "finger41", "c2": 0.0035, "c1": 0.9206, "c0": 0.6344},
        "finger52": {"driver": "finger51", "c2": 0.0035, "c1": 0.9206, "c0": 0.6344},
    },

    # How the quadratic coupling is reduced to the linear URDF <mimic>:
    #   "chord"   - line through both end points of the travel (default):
    #               exact at q=0 and q=max, worst case c2*travel^2/4 in between
    #   "tangent" - first-order term only (the old behaviour): exact slope at
    #               q=0 but under-drives by c2*travel^2 at the far end
    "mimic_fit": "chord",

    # visual material rgba per link.  The five fingertip pads are one and the
    # same black silicone part ("硅胶 A25 黑色"), but SolidWorks exports the
    # pinky pad with a different grey, so all five are pinned to the same value
    # here.  Links not listed keep whatever the exporter produced.
    "colors": {
        "finger14_link": [0.0392156862745098, 0.0392156862745098, 0.0392156862745098, 1.0],
        "finger23_link": [0.0392156862745098, 0.0392156862745098, 0.0392156862745098, 1.0],
        "finger33_link": [0.0392156862745098, 0.0392156862745098, 0.0392156862745098, 1.0],
        "finger43_link": [0.0392156862745098, 0.0392156862745098, 0.0392156862745098, 1.0],
        "finger53_link": [0.0392156862745098, 0.0392156862745098, 0.0392156862745098, 1.0],
    },
}

NDEC = 6  # decimals written into the URDF / CSV
TOL = 1e-6  # verification tolerance (half of the last written digit is 5e-7)


def fmt(value: float) -> str:
    """Format a number for the URDF, rounding -0.0 to 0.0."""
    if abs(value) < 0.5 * 10 ** -NDEC:
        value = 0.0
    return f"{value:.{NDEC}f}"


# --------------------------------------------------------------------------- #
# Derived limit table
# --------------------------------------------------------------------------- #

def build_limits(cfg: dict) -> dict:
    """Return {joint: dict(lower=, upper=, mimic=dict|None)} in radians."""
    out: dict = {}
    for joint, deg in cfg["travel_deg"].items():
        out[joint] = {"lower": 0.0, "upper": deg * DEG, "mimic": None}

    fit = cfg.get("mimic_fit", "chord")
    if fit not in ("chord", "tangent"):
        raise SystemExit(f"unknown mimic_fit {fit!r} (use 'chord' or 'tangent')")

    for joint, spec in cfg["coupling"].items():
        driver = spec["driver"]
        if driver not in cfg["travel_deg"]:
            raise SystemExit(f"coupling driver {driver!r} has no entry in travel_deg")
        x = cfg["travel_deg"][driver]
        y = spec["c2"] * x * x + spec["c1"] * x + spec["c0"]
        if fit == "chord":
            # straight line through (0, c0) and (travel, y)
            multiplier = spec["c1"] + spec["c2"] * x
            dev = spec["c2"] * x * x / 4.0
        else:
            multiplier = spec["c1"]
            dev = spec["c2"] * x * x
        out[joint] = {
            "lower": spec["c0"] * DEG,
            "upper": y * DEG,
            "mimic": {"driver": driver, "multiplier": multiplier,
                      "offset": spec["c0"] * DEG},
            "fit": fit,
            "dev_deg": dev,
        }

    # the linear mimic output must stay inside the follower limits
    for joint, lim in out.items():
        if not lim["mimic"]:
            continue
        mm = lim["mimic"]
        drv = out[mm["driver"]]
        ends = [mm["multiplier"] * drv[side] + mm["offset"] for side in ("lower", "upper")]
        if min(ends) < lim["lower"] - TOL or max(ends) > lim["upper"] + TOL:
            raise SystemExit(
                f"{joint}: mimic output {min(ends):.6f}..{max(ends):.6f} rad does not fit "
                f"inside its limit {lim['lower']:.6f}..{lim['upper']:.6f}"
            )
    return out


def describe(limits: dict) -> str:
    lines = [f"{'joint':10s} {'type':9s} {'lower':>10s} {'upper':>10s} {'deg':>8s}  mimic"]
    for joint, lim in limits.items():
        kind = "passive" if lim["mimic"] else "active"
        m = ""
        if lim["mimic"]:
            mm = lim["mimic"]
            m = f"{mm['driver']} x{mm['multiplier']} +{fmt(mm['offset'])}"
        lines.append(
            f"{joint:10s} {kind:9s} {lim['lower']:10.6f} {lim['upper']:10.6f} "
            f"{lim['upper'] / DEG:8.2f}  {m}"
        )
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# URDF patching (text based, so the SolidWorks formatting survives)
# --------------------------------------------------------------------------- #

COMMENT_BEGIN = "  <!-- @@AUTO-LIMITS BEGIN@@"
COMMENT_END = "    @@AUTO-LIMITS END@@ -->"


def documentation_block(limits: dict, nl: str, colors: dict | None = None) -> str:
    active = [j for j, v in limits.items() if not v["mimic"]]
    passive = [j for j, v in limits.items() if v["mimic"]]
    travel = " | ".join(f"{j} {limits[j]['upper'] / DEG:.0f}" for j in active)
    rows = [
        COMMENT_BEGIN,
        "    Auto-generated by tools/patch_sw2urdf_limits.py - do not edit by hand.",
        "    Regenerate with:",
        "      python3 tools/patch_sw2urdf_limits.py -p <package_dir>",
        "",
        "    Active joint travel (deg, measured from q = 0):",
        f"      {travel}",
        "",
        "    Passive followers use the README coupling formulas (quadratic, as in",
        "    the MJCF polycoef).  A URDF <mimic> can only be linear, so the",
        "    quadratic is replaced by its chord over one travel range: exact at",
        "    q=0 and at q=max, worst-case deviation c2*travel^2/4 in between.",
        "    The follower <limit> keeps the true quadratic end points, so the hard",
        "    stops still match the real linkage.",
    ]
    for j in passive:
        mm = limits[j]["mimic"]
        spec = CONFIG["coupling"].get(j, {})
        f = (f"      {j} <- {mm['driver']}: y = {spec.get('c2')}x2 + {spec.get('c1')}x "
             f"+ {spec.get('c0')}  ->  mimic x{num_fmt(mm['multiplier'])} +{fmt(mm['offset'])}")
        rows.append(f)
    if colors:
        rows += [
            "",
            "    Fingertip pads finger14/23/33/43/53 are all the same black silicone",
            "    part, so their visual colour is pinned to one rgba value as well.",
        ]
    rows.append(COMMENT_END)
    block = nl.join(rows)
    # an XML comment must not contain a double hyphen
    body = block[block.index("<!--") + 4: block.rindex("-->")]
    if "--" in body:
        raise SystemExit("internal error: comment body contains '--'")
    return block


LINK_CASE = re.compile(r'(\b(?:name|link)=")(finger\d+)_Link(")')
MESH_CASE = re.compile(r'(meshes/[^"/\\]+)_Link\.STL')


def _normalise_link_names(text: str) -> str:
    """`finger11_Link` -> `finger11_link` (SolidWorks keeps the CAD spelling)."""
    text = LINK_CASE.sub(r"\1\2_link\3", text)
    return MESH_CASE.sub(r"\1_link.STL", text)


def _joint_blocks(text: str):
    """Yield (start, end, name) for every <joint>...</joint> block."""
    for m in re.finditer(r"<joint\b.*?</joint>", text, re.S):
        name_m = re.search(r'\bname="([^"]+)"', m.group(0))
        if name_m:
            yield m.start(), m.end(), name_m.group(1)


def _clear_fixed_axis(block: str) -> str:
    """A fixed joint has no degree of freedom, so drop the axis SW writes."""
    if 'type="fixed"' not in block:
        return block
    return re.sub(r'(<axis\s+xyz=")[^"]*(")', r"\g<1>0 0 0\g<2>", block, count=1)


def _link_blocks(text: str):
    """Yield (start, end, name) for every <link>...</link> block."""
    for m in re.finditer(r"<link\b.*?</link>", text, re.S):
        name_m = re.search(r'\bname="([^"]+)"', m.group(0))
        if name_m:
            yield m.start(), m.end(), name_m.group(1)


def num_fmt(value: float, sig: int = 6) -> str:
    """Compact number for the documentation comment (0.8808 -> '0.8808')."""
    s = f"{value:.{sig}g}"
    return s.replace("e", "E")


def cstr(value: float) -> str:
    """Format a colour channel: 1.0 -> '1', 0.0392156862745098 -> as exported."""
    s = repr(float(value))
    return s[:-2] if s.endswith(".0") else s


def rgba_str(channels) -> str:
    return " ".join(cstr(c) for c in channels)


def _set_color(block: str, channels, nl: str) -> str:
    """Rewrite the rgba of the <visual> material inside one link block."""
    m = re.search(r"<visual>.*?</visual>", block, re.S)
    if not m:
        return block
    vis = m.group(0)
    rgba = rgba_str(channels)

    new_vis, n = re.subn(r'(<color\s+rgba=")[^"]*(")',
                         lambda mm: mm.group(1) + rgba + mm.group(2), vis, count=1)
    if n == 0:
        # no <color> element yet: attach one to the visual material
        mat = re.search(r"<material\b[^>]*?/>", vis, re.S)
        if mat:
            name_m = re.search(r'name="([^"]*)"', mat.group(0))
            name = name_m.group(1) if name_m else ""
            indent = re.search(r"\n([ \t]*)<material", vis)
            ind = indent.group(1) if indent else "      "
            new_mat = (f'{ind}<material{nl}{ind}  name="{name}">{nl}'
                       f'{ind}  <color{nl}{ind}    rgba="{rgba}" />{nl}{ind}</material>')
            new_vis = vis[:mat.start()] + new_mat + vis[mat.end():]
        else:
            close = vis.find("</material>")
            if close < 0:  # pragma: no cover
                return block
            indent = re.search(r"\n([ \t]*)<color", vis)
            ind = indent.group(1) if indent else "        "
            new_vis = vis[:close] + f'{ind}<color{nl}{ind}  rgba="{rgba}" />{nl}    ' + vis[close:]
    return block[:m.start()] + new_vis + block[m.end():]


def _set_limit(block: str, lower: float, upper: float, nl: str) -> str:
    """Rewrite lower/upper inside the <limit ...> element of one joint block."""
    if 'type="fixed"' in block:
        return block  # a fixed joint needs no travel range
    start = block.find("<limit")
    if start < 0:
        # no limit element: create one in the canonical SolidWorks shape
        indent = re.search(r"\n([ \t]*)<axis", block)
        ind = indent.group(1) if indent else "    "
        limit = (f"{ind}<limit{nl}{ind}  lower=\"{fmt(lower)}\"{nl}"
                 f"{ind}  upper=\"{fmt(upper)}\"{nl}{ind}  effort=\"0\"{nl}"
                 f"{ind}  velocity=\"0\" />{nl}")
        return block.replace(nl + "  </joint>", nl + limit + "  </joint>", 1)

    end = block.find("/>", start) + 2
    elem = block[start:end]
    if 'lower="' in elem and 'upper="' in elem:
        # in-place substitution keeps whatever layout SolidWorks produced
        new = re.sub(r'(\blower=")[^"]*(")', lambda m: m.group(1) + fmt(lower) + m.group(2), elem)
        new = re.sub(r'(\bupper=")[^"]*(")', lambda m: m.group(1) + fmt(upper) + m.group(2), new)
        return block[:start] + new + block[end:]
    # one of the attributes is missing: rebuild the element in the canonical shape
    indent_m = re.search(r"\n([ \t]*)<limit", block)
    ind = indent_m.group(1) if indent_m else "    "
    canonical = (f'{ind}<limit{nl}{ind}  lower="{fmt(lower)}"{nl}{ind}  upper="{fmt(upper)}"{nl}'
                 f'{ind}  effort="0"{nl}{ind}  velocity="0" />')
    return block[:start] + canonical + block[end:]


def _set_mimic(block: str, mm: dict, nl: str) -> str:
    """Insert or rewrite the <mimic> element of one joint block."""
    indent_m = re.search(r"\n([ \t]*)<limit", block)
    ind = indent_m.group(1) if indent_m else "    "
    body = (f"{ind}<mimic{nl}"
            f"{ind}  joint=\"{mm['driver']}\"{nl}"
            f"{ind}  multiplier=\"{fmt(mm['multiplier'])}\"{nl}"
            f"{ind}  offset=\"{fmt(mm['offset'])}\" />")
    existing = re.search(r"[ \t]*<mimic\b.*?/>", block, re.S)
    if existing:
        start = existing.start()
        # keep the leading newline/indentation that precedes the element
        return block[:start] + body + block[existing.end():]
    limit_end = block.find("/>", block.find("<limit")) + 2
    if limit_end < 2:  # pragma: no cover - no <limit> at all
        return block.replace(nl + "  </joint>", nl + body + nl + "  </joint>", 1)
    return block[:limit_end] + nl + body + block[limit_end:]


def _set_comment(text: str, limits: dict, nl: str, colors: dict | None = None) -> str:
    block = documentation_block(limits, nl, colors)
    m = re.search(re.escape(COMMENT_BEGIN) + r".*?" + re.escape(COMMENT_END), text, re.S)
    if m:
        return text[:m.start()] + block + text[m.end():]
    robot = re.search(r"<robot\b[^>]*>", text)
    if not robot:  # pragma: no cover
        return text
    return text[:robot.end()] + nl + block + text[robot.end():]


def patch_urdf_text(text: str, limits: dict, colors: dict | None = None) -> str:
    # Work on LF internally and convert once at the end.  Converting the new
    # fragments to CRLF up front would double the CR on the final pass.
    crlf = "\r\n" in text
    flat = _normalise_link_names(text.replace("\r\n", "\n"))
    nl = "\n"

    out, last = [], 0
    for start, end, name in _joint_blocks(flat):
        block = flat[start:end]
        # fixed joints are not in `limits`, but their bogus axis is cleaned too
        new_block = _clear_fixed_axis(block)
        if name in limits:
            lim = limits[name]
            new_block = _set_limit(new_block, lim["lower"], lim["upper"], nl)
            if lim["mimic"]:
                new_block = _set_mimic(new_block, lim["mimic"], nl)
        if new_block == block:
            continue
        out.append(flat[last:start])
        out.append(new_block)
        last = end
    out.append(flat[last:])
    flat = "".join(out)

    if colors:
        seen = set()
        out, last = [], 0
        for start, end, name in _link_blocks(flat):
            if name not in colors:
                continue
            seen.add(name)
            out.append(flat[last:start])
            out.append(_set_color(flat[start:end], colors[name], nl))
            last = end
        out.append(flat[last:])
        flat = "".join(out)
        missing = set(colors) - seen
        if missing:
            raise SystemExit("links missing from URDF: " + ", ".join(sorted(missing)))

    flat = _set_comment(flat, limits, nl, colors)
    return flat.replace("\n", "\r\n") if crlf else flat


# --------------------------------------------------------------------------- #
# CSV syncing
# --------------------------------------------------------------------------- #

def patch_csv_text(text: str, limits: dict, colors: dict | None = None) -> tuple[str, list[str]]:
    nl = "\r\n" if "\r\n" in text else "\n"
    rows = list(csv.reader(io.StringIO(text, newline="")))
    if not rows:
        raise SystemExit("empty CSV")
    head = rows[0]
    try:
        i_name = head.index("Joint Name")
        i_lo = head.index("Limit Lower")
        i_hi = head.index("Limit Upper")
    except ValueError as exc:  # pragma: no cover
        raise SystemExit(f"unexpected CSV header: {exc}")
    i_link = head.index("Link Name")
    i_rgba = [head.index(c) for c in ("Color Red", "Color Green", "Color Blue", "Color Alpha")]
    i_mesh = [head.index(c) for c in ("Mesh Filename", "Collision Mesh Filename") if c in head]

    changed = []
    seen = set()
    seen_links = set()
    for row in rows[1:]:
        if len(row) <= max(i_name, i_lo, i_hi):
            continue

        # limits
        name = row[i_name]
        if name in limits:
            seen.add(name)
            lo, hi = fmt(limits[name]["lower"]), fmt(limits[name]["upper"])
            if row[i_lo] != lo or row[i_hi] != hi:
                changed.append(f"{name}: {row[i_lo]}/{row[i_hi]} -> {lo}/{hi}")
                row[i_lo], row[i_hi] = lo, hi

        # link-name / mesh-path normalisation
        if LINK_CASE.search(f'name="{row[i_link]}"'):
            row[i_link] = _normalise_link_names(f'name="{row[i_link]}"')[6:-1]
        for k in i_mesh:
            if MESH_CASE.search(row[k]):
                row[k] = MESH_CASE.sub(r"\1_link.STL", row[k])

        # colours
        link = row[i_link]
        if colors and link in colors:
            seen_links.add(link)
            want = [cstr(c) for c in colors[link]]
            have = [row[k] for k in i_rgba]
            if have != want:
                changed.append(f"{link}: color {have} -> {want}")
                for k, v in zip(i_rgba, want):
                    row[k] = v

    missing = set(limits) - seen
    if missing:
        raise SystemExit("joints missing from CSV: " + ", ".join(sorted(missing)))
    if colors:
        missing_links = set(colors) - seen_links
        if missing_links:
            raise SystemExit("links missing from CSV: " + ", ".join(sorted(missing_links)))

    buf = io.StringIO()
    csv.writer(buf, lineterminator=nl).writerows(rows)
    return buf.getvalue(), changed



# --------------------------------------------------------------------------- #
# Package tidy-up (line endings + export junk)
# --------------------------------------------------------------------------- #

TEXT_EXT = {".urdf", ".csv", ".xml", ".yaml", ".yml", ".launch", ".py",
            ".txt", ".md", ".rviz", ".json"}
JUNK_NAMES = {"export.log", ".DS_Store"}


def tidy_package(pkg: Path, write: bool = True) -> list[str]:
    """CRLF -> LF for text files, drop SolidWorks/macOS junk, remove empty dirs."""
    msgs: list[str] = []
    meshes = pkg / "meshes"
    if meshes.is_dir():
        for f in sorted(meshes.iterdir()):
            if not f.is_file() or not f.name.endswith("_Link.STL"):
                continue
            target = f.with_name(f.name[: -len("_Link.STL")] + "_link.STL")
            # on a case-insensitive filesystem `target` already "exists" for the
            # very same file, so compare identity rather than existence
            if target.exists() and not os.path.samefile(f, target):
                continue
            msgs.append(f"rename meshes/{f.name} -> {target.name}")
            if write:
                tmp = f.with_name(f.name + ".rename.tmp")
                f.rename(tmp)
                tmp.rename(target)
    for path in sorted(pkg.rglob("*")):
        if not path.is_file():
            continue
        rel = path.relative_to(pkg)
        if path.name in JUNK_NAMES:
            msgs.append(f"delete {rel}")
            if write:
                path.unlink()
            continue
        if path.suffix.lower() not in TEXT_EXT:
            continue
        data = path.read_bytes()
        if b"\r\n" in data:
            msgs.append(f"crlf->lf {rel}")
            if write:
                path.write_bytes(data.replace(b"\r\n", b"\n"))
    for d in sorted(pkg.rglob("*")):
        if d.is_dir() and d.name in {"textures"} and not any(d.iterdir()):
            msgs.append(f"rmdir {d.relative_to(pkg)}")
            if write:
                d.rmdir()
    return msgs


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def load_config(path: Path | None) -> dict:
    cfg = json.loads(json.dumps(CONFIG))  # deep copy
    if path:
        override = json.loads(path.read_text(encoding="utf-8"))
        for key, value in override.items():
            if key in ("travel_deg", "coupling", "colors"):
                cfg[key].update(value)
            else:
                cfg[key] = value
    if cfg["robot"] != CONFIG["robot"]:
        pass  # allowed: script also works for other hands
    return cfg


def find_targets(pkg: Path, urdf: Path | None, csv_path: Path | None) -> tuple[Path, Path | None]:
    urdf_dir = pkg / "urdf"
    if urdf is None:
        found = sorted(urdf_dir.glob("*.urdf"))
        if len(found) != 1:
            raise SystemExit(f"expected exactly one .urdf in {urdf_dir}, found {len(found)}")
        urdf = found[0]
    if csv_path is None:
        cand = urdf.with_suffix(".csv")
        csv_path = cand if cand.exists() else None
    return urdf, csv_path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--package", type=Path, required=True,
                    help="ROS package folder containing urdf/ (and meshes/)")
    ap.add_argument("--urdf", type=Path, help="explicit URDF path")
    ap.add_argument("--csv", type=Path, help="explicit CSV path")
    ap.add_argument("--no-csv", action="store_true", help="do not touch the CSV")
    ap.add_argument("--tidy", action="store_true",
                    help="also convert every text file to LF and delete export.log / .DS_Store")
    ap.add_argument("--no-colors", action="store_true",
                    help="leave the visual colours exactly as exported "
                         "(also drops the colour note from the header comment)")
    ap.add_argument("--config", type=Path, help="JSON file overriding travel_deg / coupling")
    ap.add_argument("--dry-run", action="store_true", help="show the result, write nothing")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if anything would change (for CI)")
    ap.add_argument("-q", "--quiet", action="store_true", help="only print the summary line")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    limits = build_limits(cfg)
    colors = {} if args.no_colors else (cfg.get("colors") or {})

    urdf_path, csv_path = find_targets(args.package, args.urdf, args.csv)
    raw_urdf = urdf_path.read_bytes().decode("utf-8")
    new_urdf = patch_urdf_text(raw_urdf, limits, colors)

    urdf_changed = new_urdf != raw_urdf
    csv_changes: list[str] = []
    new_csv = None
    if csv_path and not args.no_csv:
        raw_csv = csv_path.read_bytes().decode("utf-8")
        new_csv, csv_changes = patch_csv_text(raw_csv, limits, colors)

    tidy_msgs: list[str] = []
    if args.tidy:
        tidy_msgs = tidy_package(args.package, write=False)

    changed = urdf_changed or bool(csv_changes) or bool(tidy_msgs)
    if not args.quiet:
        print(describe(limits))
        if colors:
            for link, rgba in colors.items():
                print(f"colour {link:14s} {rgba_str(rgba)}")
        print()

    if args.check:
        if changed:
            parts = []
            if urdf_changed:
                parts.append(str(urdf_path))
            if csv_changes:
                parts.append(str(csv_path))
            if tidy_msgs:
                shown = ", ".join(tidy_msgs[:3])
                more = f" (+{len(tidy_msgs) - 3} more)" if len(tidy_msgs) > 3 else ""
                parts.append(f"tidy: {shown}{more}")
            print("OUT OF SYNC: " + "; ".join(parts), file=sys.stderr)
            return 1
        print("in sync")
        return 0

    if args.dry_run:
        if urdf_changed:
            print(f"[dry-run] would rewrite {urdf_path}")
        if csv_changes:
            print(f"[dry-run] would rewrite {csv_path}:")
            for line in csv_changes:
                print("   ", line)
        for line in tidy_msgs:
            print(f"[dry-run] would {line}")
        if not changed:
            print("nothing to do")
        return 0

    if urdf_changed:
        urdf_path.write_bytes(new_urdf.encode("utf-8"))
        print(f"patched {urdf_path}")
    else:
        print(f"unchanged {urdf_path}")

    if csv_changes:
        csv_path.write_bytes(new_csv.encode("utf-8"))
        print(f"patched {csv_path}")
        for line in csv_changes:
            print("   ", line)
    elif csv_path and not args.no_csv:
        print(f"unchanged {csv_path}")

    if args.tidy:
        done = tidy_package(args.package, write=True)
        if done:
            for line in done:
                print(f"  {line}")
        else:
            print("package already tidy")

    # final sanity check: the file must still parse and match the table
    root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in limits:
            continue
        lim = joint.find("limit")
        lo, hi = float(lim.get("lower")), float(lim.get("upper"))
        exp = limits[name]
        if abs(lo - exp["lower"]) > TOL or abs(hi - exp["upper"]) > TOL:
            raise SystemExit(f"verification failed for {name}: {lo}, {hi}")
        mi = joint.find("mimic")
        if exp["mimic"]:
            if mi is None:
                raise SystemExit(f"verification failed: {name} has no <mimic>")
            if mi.get("joint") != exp["mimic"]["driver"] or \
               abs(float(mi.get("multiplier")) - exp["mimic"]["multiplier"]) > TOL or \
               abs(float(mi.get("offset")) - exp["mimic"]["offset"]) > TOL:
                raise SystemExit(f"verification failed: bad <mimic> on {name}")
        elif mi is not None:
            raise SystemExit(f"verification failed: unexpected <mimic> on {name}")
    for link in root.findall("link"):
        if link.get("name") not in colors:
            continue
        col = link.find("visual/material/color")
        if col is None or col.get("rgba") != rgba_str(colors[link.get("name")]):
            raise SystemExit(f"verification failed: colour of {link.get('name')}")
    print("verification: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
