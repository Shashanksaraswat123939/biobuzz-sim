"""
Tessellate cad/field-cad-step.step into render meshes, grouped by role.

    python tools/cad2assets.py            # normal
    python tools/cad2assets.py --fast     # coarser, for a quick look

Writes assets/field.glb with one mesh per role (frame, rocker-red, rocker-blue, flower-1..4,
perimeter, tiles) in METRES, Y up, ready for THREE.GLTFLoader.

This is PLAN.md section 6, which was skipped at first and is now real: the renderer draws the
actual AndyMark geometry instead of my reconstruction of it. The PHYSICS still uses the convex
boxes in packages/core/src/field/geometry.ts -- a physics engine turns a concave mesh into its
hull, and a hull across the CELL mouth is the "balls float on an invisible lid" bug. So this
pipeline deliberately produces render geometry only.

Balls are NOT exported: they are instanced spheres, and their staged positions come from
tools/cad2staging.mjs.
"""
import json
import math
import re
import sys
import time

import numpy as np
import trimesh

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB

MM_TO_M = 0.001
FAST = "--fast" in sys.argv

# Deflection in mm. Coarser on the big static furniture, finer where shape reads.
# Tuned so the whole field lands around 350k triangles: fine where shape reads (the CELL
# pocket, the FLOWER pipes), coarse on flat furniture.
DEFLECTION = {"rocker": 2.0, "flower": 2.2, "frame": 5.0, "perimeter": 9.0}
if FAST:
    DEFLECTION = {k: v * 2.5 for k, v in DEFLECTION.items()}

# Fasteners and cable ties are most of the part count and none of the silhouette.
MIN_VOL_MM3 = 900.0

# This STEP carries no colour data at all (checked: 0 of 171 parts), so colour comes from
# what each part IS. Alliance-coloured plastic for the CELL skins and ribs, bare aluminium
# for structure, white for the AprilTag panels and the HIPS flower pipes.
ALUMINIUM = (176, 184, 194)
BY_NAME = [
    (re.compile(r"april tag", re.I), (246, 248, 250)),
    (re.compile(r"goal rib|skin|basket", re.I), None),          # None = the alliance colour
    (re.compile(r"churro|tube|bar|bracket|holder|foot|leg|corner|panel", re.I), ALUMINIUM),
    (re.compile(r"hips pipe|flower layer|backstop|peanut", re.I), (238, 243, 248)),
    (re.compile(r"glass", re.I), (150, 205, 232)),
    (re.compile(r"damper", re.I), (60, 66, 76)),
]
RED = (196, 48, 48)
BLUE = (40, 96, 200)
DEFAULT_RGB = {
    "rocker": ALUMINIUM,
    "frame": (150, 158, 170),
    "flowers": (238, 243, 248),
    "perimeter": (150, 205, 232),
}


def colour_for(leaf: str, role: str):
    """What colour this part should be, given the STEP gives us none."""
    alliance = RED if role.endswith("red") else BLUE if role.endswith("blue") else ALUMINIUM
    for rx, rgb in BY_NAME:
        if rx.search(leaf):
            return alliance if rgb is None else rgb
    return DEFAULT_RGB.get(role.split("-")[0], ALUMINIUM)
SKIP = re.compile(
    r"screw|nut\b|washer|rivnut|rivet|cable tie|spacer|bearing|plug|wing nut|standoff",
    re.I,
)

# PLAN.md Appendix A, as an ordered list: first match wins.
ROLES = [
    ("rocker-red", re.compile(r"am-5853.*red hive|red cell|red goal|am-5865-red", re.I)),
    ("rocker-blue", re.compile(r"am-5853.*blue hive|blue cell|blue goal|am-5865-blue", re.I)),
    ("frame", re.compile(r"am-5854|a-frame|acm panel|foot bar|frame foot|under tile bar|pivot|damper|axle holder", re.I)),
    ("flower", re.compile(r"am-5855|flower|peanut support", re.I)),  # all four, merged
    ("perimeter", re.compile(r"field side glass|am-2556a|riveted ftc panel|corner hinge|am-0481b", re.I)),
    # Tiles are deliberately NOT exported: they are flat squares, the renderer already draws
    # them procedurally, and they were 80k triangles of nothing.
]


def role_of(path: str) -> str | None:
    for name, rx in ROLES:
        if rx.search(path):
            return "flowers" if name == "flower" else name
    return None


def name_of(lbl) -> str:
    n = TDataStd_Name()
    try:
        if lbl.FindAttribute(TDataStd_Name.GetID_s(), n):
            return n.Get().ToExtString()
    except Exception:
        pass
    return ""


def triangles(shape, deflection):
    """Tessellate a shape and return (vertices Nx3 mm, faces Mx3)."""
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)
    verts, faces = [], []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(verts)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X(), p.Y(), p.Z()))
            reversed_face = face.Orientation() == 1  # TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if reversed_face:
                    b, c = c, b
                faces.append((base + a - 1, base + b - 1, base + c - 1))
        ex.Next()
    return verts, faces


def main() -> None:
    t0 = time.time()
    step = "cad/field-cad-step.step"
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    rdr = STEPCAFControl_Reader()
    rdr.SetNameMode(True)
    rdr.SetColorMode(True)   # the STEP carries per-part colour; use it
    assert rdr.ReadFile(step) == IFSelect_RetDone, "STEP read failed"
    print(f"read {step} ({time.time() - t0:.0f}s)", flush=True)
    rdr.Transfer(doc)
    print(f"transfer ok ({time.time() - t0:.0f}s)", flush=True)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ctool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def colour_of(label, shape):
        """sRGB 0-255 for a part, falling back through the colour types the STEP may use."""
        col = Quantity_Color()
        for ct in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen, XCAFDoc_ColorType.XCAFDoc_ColorCurv):
            try:
                if ctool.GetColor(label, ct, col) or ctool.GetColor(shape, ct, col):
                    r, g, b = col.Red(), col.Green(), col.Blue()
                    return (int(r * 255), int(g * 255), int(b * 255))
            except Exception:
                pass
        return None

    buckets: dict[str, list] = {}
    stats = {"parts": 0, "skipped": 0}

    def walk(lbl, loc, path):
        nm = name_of(lbl)
        ref = TDF_Label()
        if tool.IsReference_s(lbl):
            tool.GetReferredShape_s(lbl, ref)
            here = loc.Multiplied(tool.GetLocation_s(lbl))
            target = ref
        else:
            here, target = loc, lbl
        p = path + [nm or name_of(target)]
        if tool.IsAssembly_s(target):
            comps = TDF_LabelSequence()
            tool.GetComponents_s(target, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), here, p)
            return

        full = "/".join(p)
        role = role_of(full)
        if role is None:
            return
        leaf = p[-1]
        if SKIP.search(leaf):
            stats["skipped"] += 1
            return

        shp = tool.GetShape_s(target)
        if shp.IsNull():
            return
        shp = shp.Moved(here)

        # Drop anything too small to see.
        vol = 0.0
        ex = TopExp_Explorer(shp, TopAbs_SOLID)
        while ex.More():
            g = GProp_GProps()
            BRepGProp.VolumeProperties_s(ex.Current(), g)
            vol += g.Mass()
            ex.Next()
        if vol < MIN_VOL_MM3:
            stats["skipped"] += 1
            return

        v, f = triangles(shp, DEFLECTION.get(role.split("-")[0], 4.0))
        if f:
            rgb = colour_of(target, shp) or colour_for(leaf, role)
            buckets.setdefault(role, []).append((v, f, rgb))
            stats["parts"] += 1
            stats.setdefault("coloured", 0)
            if colour_of(target, shp):
                stats["coloured"] += 1

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    print(f"tessellated {stats['parts']} parts ({stats.get('coloured', 0)} with STEP colour), "
          f"skipped {stats['skipped']} ({time.time() - t0:.0f}s)", flush=True)

    scene = trimesh.Scene()
    index = {}
    for role, pieces in sorted(buckets.items()):
        vs, fs, cs, off = [], [], [], 0
        for v, f, rgb in pieces:
            vs.extend(v)
            cs.extend([(*rgb, 255)] * len(v))
            fs.extend([(a + off, b + off, c + off) for a, b, c in f])
            off += len(v)
        verts = np.asarray(vs, dtype=np.float64) * MM_TO_M
        faces = np.asarray(fs, dtype=np.int64)
        # process=False keeps the vertex/colour arrays aligned -- merging would drop colours.
        mesh = trimesh.Trimesh(vertices=verts, faces=faces, vertex_colors=np.asarray(cs, dtype=np.uint8), process=False)
        scene.add_geometry(mesh, node_name=role, geom_name=role)
        index[role] = {"parts": len(pieces), "tris": int(len(mesh.faces))}
        print(f"  {role:14s} {len(pieces):4d} parts  {len(mesh.faces):7d} tris", flush=True)

    total = sum(v["tris"] for v in index.values())
    scene.export("assets/field.glb")
    with open("assets/field-index.json", "w", encoding="utf-8") as fh:
        json.dump({"units": "m, Y up, world frame", "source": step, "groups": index, "tris": total}, fh, indent=1)
    print(f"wrote assets/field.glb  {total} triangles total ({time.time() - t0:.0f}s)")
    if total > 900_000:
        print("  (heavy -- re-run with --fast, or raise DEFLECTION)")


if __name__ == "__main__":
    main()
