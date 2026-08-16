#!/usr/bin/env python3
"""Tile powerplant.obj into a 3x3 grid (9 copies, 114.8M tris) for
ultra-large-model stress tests.

Step 1 rewrites the file with absolute (1-based) face indices, since
negative-relative indices would resolve across tile boundaries after
duplication. Step 2 emits N tiles with a per-tile position offset and
a per-tile index base.
"""
import sys, time

SRC = "powerplant/powerplant.obj"
ABS = "powerplant_abs.obj"
OUT = "powerplant_x9.obj"
NX, NZ = 3, 3                    # tile grid
DX, DZ = 700000.0, 220000.0      # spacing (model x-span ~611k, z ~186k)

def step1():
    nv = nvn = nvt = 0
    t0 = time.time()
    with open(SRC, "rb") as fin, open(ABS, "wb") as fout:
        for line in fin:
            if line.startswith(b"v "):
                nv += 1
                fout.write(line)
            elif line.startswith(b"vn "):
                nvn += 1
                fout.write(line)
            elif line.startswith(b"vt "):
                nvt += 1
                fout.write(line)
            elif line.startswith(b"f "):
                out = b"f"
                for tok in line.split()[1:]:
                    seg = tok.split(b"/")
                    slots = []
                    for i in range(3):
                        if i < len(seg) and seg[i]:
                            cnt = (nv, nvt, nvn)[i]
                            abs_i = int(seg[i])
                            if abs_i < 0:
                                # -k = k-th from the end; 1-based abs =
                                # cnt - k + 1 (cnt = definitions so far)
                                abs_i = cnt + abs_i + 1
                            slots.append(str(abs_i).encode())
                        else:
                            slots.append(b"")
                    while slots and slots[-1] == b"":
                        slots.pop()
                    out += b" " + b"/".join(slots)
                fout.write(out + b"\n")
            else:
                fout.write(line)
    print(f"step1: {nv} v, {nvn} vn, {nvt} vt in {time.time()-t0:.1f}s")

def step2():
    t0 = time.time()
    nv = 0
    with open(ABS, "rb") as fin, open(OUT, "wb") as fout:
        lines = fin.readlines()
        # base counts of one tile
        tile_v = sum(1 for l in lines if l.startswith(b"v "))
        tile_vn = sum(1 for l in lines if l.startswith(b"vn "))
        tile_vt = sum(1 for l in lines if l.startswith(b"vt "))
        for ty in range(NZ):
            for tx in range(NX):
                vbase = (ty * NX + tx) * tile_v
                vnbase = (ty * NX + tx) * tile_vn
                vtbase = (ty * NX + tx) * tile_vt
                dx, dz = tx * DX, ty * DZ
                for line in lines:
                    if line.startswith(b"v "):
                        p = line.split()
                        x = float(p[1]) + dx
                        z = float(p[3]) + dz
                        fout.write(b"v %f %s %f\n" % (x, p[2], z))
                    elif line.startswith(b"vn ") or line.startswith(b"vt "):
                        fout.write(line)
                    elif line.startswith(b"f "):
                        out = b"f"
                        for tok in line.split()[1:]:
                            seg = tok.split(b"/")
                            slots = []
                            for i in range(3):
                                if i < len(seg) and seg[i]:
                                    base = (vbase, vtbase, vnbase)[i]
                                    slots.append(
                                        str(int(seg[i]) + base).encode())
                                else:
                                    slots.append(b"")
                            while slots and slots[-1] == b"":
                                slots.pop()
                            out += b" " + b"/".join(slots)
                        fout.write(out + b"\n")
                    else:
                        fout.write(line)
                nv += tile_v
    print(f"step2: {nv} v total in {time.time()-t0:.1f}s")

if __name__ == "__main__":
    step1()
    step2()
