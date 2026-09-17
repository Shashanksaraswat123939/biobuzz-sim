import json, sys, math, time
t0=time.time()
STEP=sys.argv[1]; OUT=sys.argv[2]
IN=1/25.4  # OCP imports STEP in mm -> inches
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TCollection import TCollection_ExtendedString
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopLoc import TopLoc_Location
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.gp import gp_Trsf

app=XCAFApp_Application.GetApplication_s()
doc=TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"),doc)
rdr=STEPCAFControl_Reader(); rdr.SetNameMode(True); rdr.SetColorMode(False)
st=rdr.ReadFile(STEP)
assert st==IFSelect_RetDone, "read failed"
print("read ok", round(time.time()-t0,1),"s", flush=True)
rdr.Transfer(doc)
print("transfer ok", round(time.time()-t0,1),"s", flush=True)
shapeTool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

def name_of(lbl):
    n=TDataStd_Name(); 
    try:
        if lbl.FindAttribute(TDataStd_Name.GetID_s(), n):
            return n.Get().ToExtString()
    except Exception: pass
    return ""

rows=[]
def walk(lbl, loc, path, depth):
    nm=name_of(lbl)
    ref=TDF_Label()
    isref=shapeTool.IsReference_s(lbl)
    if isref:
        shapeTool.GetReferredShape_s(lbl, ref)
        thisloc = loc.Multiplied(shapeTool.GetLocation_s(lbl))
        target=ref
    else:
        thisloc=loc; target=lbl
    tname=name_of(target) or nm
    p=path+[nm or tname]
    if shapeTool.IsAssembly_s(target):
        comps=TDF_LabelSequence(); shapeTool.GetComponents_s(target, comps)
        for i in range(1, comps.Length()+1):
            walk(comps.Value(i), thisloc, p, depth+1)
    else:
        shp=shapeTool.GetShape_s(target)
        if shp.IsNull(): return
        shp=shp.Moved(thisloc)
        ex=TopExp_Explorer(shp, TopAbs_SOLID); nsol=0
        vol=0.0; cx=cy=cz=0.0
        bb=Bnd_Box(); BRepBndLib.Add_s(shp, bb, False)
        while ex.More():
            s=ex.Current(); g=GProp_GProps(); BRepGProp.VolumeProperties_s(s,g)
            v=g.Mass(); c=g.CentreOfMass(); vol+=v; cx+=c.X()*v; cy+=c.Y()*v; cz+=c.Z()*v; nsol+=1; ex.Next()
        if vol>0: cx/=vol; cy/=vol; cz/=vol
        try:
            x0,y0,z0,x1,y1,z1=bb.Get()
        except Exception:
            x0=y0=z0=x1=y1=z1=float('nan')
        rows.append(dict(path="/".join(p), name=tname, depth=depth, solids=nsol,
            vol_in3=vol*IN**3, centroid_in=[cx*IN,cy*IN,cz*IN],
            bbox_in=[x0*IN,y0*IN,z0*IN,x1*IN,y1*IN,z1*IN]))

roots=TDF_LabelSequence(); shapeTool.GetFreeShapes(roots)
print("roots", roots.Length(), flush=True)
for i in range(1, roots.Length()+1):
    walk(roots.Value(i), TopLoc_Location(), [], 0)
print("leaves", len(rows), round(time.time()-t0,1),"s", flush=True)
json.dump(rows, open(OUT,"w"), indent=1)

# ---- aggregates ----
def agg(pred, label):
    sel=[r for r in rows if pred(r['path'])]
    V=sum(r['vol_in3'] for r in sel)
    if V==0: print(label,"(none)"); return
    c=[sum(r['centroid_in'][k]*r['vol_in3'] for r in sel)/V for k in range(3)]
    xs=[r['bbox_in'] for r in sel if not math.isnan(r['bbox_in'][0])]
    bb=[min(b[0] for b in xs),min(b[1] for b in xs),min(b[2] for b in xs),max(b[3] for b in xs),max(b[4] for b in xs),max(b[5] for b in xs)]
    print(f"\n[{label}] parts={len(sel)} vol={V:.1f} in^3 centroid_in={[round(x,2) for x in c]} bbox_in={[round(x,2) for x in bb]}")
    for r in sorted(sel, key=lambda r:-r['vol_in3'])[:14]:
        print(f"   {r['vol_in3']:8.1f} in3  c={[round(x,1) for x in r['centroid_in']]}  bb={[round(x,1) for x in r['bbox_in']]}  {r['name'][:60]}")


DENS={ # g/cm3, guesses by part name; flagged in output
 'Goal Rib':('HDPE/PC sheet',1.05),'Bottom Skin':('polycarbonate',1.20),'Top Skin':('polycarbonate',1.20),'Back Skin':('polycarbonate',1.20),
 'Basket Base Tube':('aluminium tube',2.70),'Churro':('aluminium extrusion',2.70),'April Tag':('vinyl/PVC sticker',1.30),
 'Press In Plug':('nylon',1.15),'Cable Tie':('nylon',1.15),'Socket head':('steel',7.85),'Screw':('steel',7.85),'Nut':('steel',7.85),
 'Washer':('steel',7.85),'Rivnut':('steel',7.85),'Bolt':('steel',7.85),'Spacer':('aluminium',2.70),'Stepped Sp':('aluminium',2.70),
 'Axle':('steel',7.85),'Damper':('n/a - frame side',0.0)}
def dens(name):
    for k,(m,d) in DENS.items():
        if k.lower() in name.lower(): return m,d
    return 'UNKNOWN (assumed 1.2)',1.20
def massprops(pred,label):
    sel=[r for r in rows if pred(r['path'])]
    M=0; c=[0,0,0]; unk=[]
    for r in sel:
        m,d=dens(r['name']); mass=r['vol_in3']*16.387*d/1000.0  # kg
        if 'UNKNOWN' in m: unk.append((r['name'],round(mass,3)))
        M+=mass; c=[c[k]+r['centroid_in'][k]*mass for k in range(3)]
    c=[x/M for x in c] if M else c
    print(f"\n[MASS-GUESS {label}] mass={M:.2f} kg  CG_in={[round(x,2) for x in c]}  (pivot axis: Y=43.95 in, Z=0, along X)")
    print(f"   CG offset from pivot: dY={c[1]-43.95:+.2f} in  dZ={c[2]:+.2f} in  -> lever={((c[1]-43.95)**2+c[2]**2)**0.5:.2f} in, angle from +Z={__import__('math').degrees(__import__('math').atan2(c[1]-43.95,c[2])):.1f} deg")
    bym={}
    for r in sel:
        m,d=dens(r['name']); bym[m]=bym.get(m,0)+r['vol_in3']*16.387*d/1000.0
    for m,v in sorted(bym.items(),key=lambda kv:-kv[1]): print(f"   {v:6.2f} kg  {m}")
    if unk: print("   UNKNOWN material parts:", unk[:12])
agg(lambda p: 'Red Hive' in p, 'RED HIVE (rocker) ')
massprops(lambda p: 'Red Hive' in p, 'RED HIVE')
agg(lambda p: 'Blue Hive' in p, 'BLUE HIVE (rocker)')
agg(lambda p: 'Frame <1>' in p and 'Hive' not in p, 'FRAME')
agg(lambda p: 'Goal Pivot' in p, 'GOAL PIVOT ASSEMBLIES')
agg(lambda p: 'Axle Holder' in p, 'AXLE HOLDERS')
agg(lambda p: 'Blumotion' in p, 'DAMPERS')
agg(lambda p: 'Cell (Audience)' in p and 'Red' in p, 'RED CELL AUDIENCE')
agg(lambda p: 'Cell (Scoring)' in p and 'Red' in p, 'RED CELL SCORING')
agg(lambda p: 'Flower Assembly <1>' in p, 'FLOWER 1')
agg(lambda p: 'Flower Backstop' in p, 'FLOWER BACKSTOPS')
agg(lambda p: 'Pollen' in p, 'POLLEN')
agg(lambda p: 'Nectar' in p, 'NECTAR')
agg(lambda p: 'Soft Tiles' in p, 'TILES')
agg(lambda p: 'Perimeter' in p or 'Riveted FTC Panel' in p, 'PERIMETER')
agg(lambda p: 'April Tag' in p, 'APRILTAG SKINS')
print("\ndone", round(time.time()-t0,1),"s")
