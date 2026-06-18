#!/usr/bin/env python3
"""
Procedurally generate STL meshes for Diablo-style visual markers.

Produces three meshes:
  - selection_ring.stl  : thin ring for selection ring under robot_1
  - reticle.stl         : larger ring for targeting reticle on ground around a tank
  - beam.stl            : tall thin cylinder for the loot beam over a tank

All meshes are written as ASCII STL.

Usage:
  ./generate_ring_mesh.py --out-dir /tmp/diablo_meshes/
"""

import argparse
import math
from pathlib import Path
from typing import List, Tuple

Vec = Tuple[float, float, float]


def normal(v1: Vec, v2: Vec, v3: Vec) -> Vec:
    """Compute outward-pointing normal for a triangle (right-hand rule)."""
    ax, ay, az = v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]
    bx, by, bz = v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]
    nx = ay * bz - az * by
    ny = az * bx - ax * bz
    nz = ax * by - ay * bx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length < 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx / length, ny / length, nz / length)


def write_stl(triangles: List[Tuple[Vec, Vec, Vec]],
              solid_name: str, out_path: Path) -> None:
    """Write ASCII STL file."""
    with out_path.open('w') as f:
        f.write(f'solid {solid_name}\n')
        for tri in triangles:
            v1, v2, v3 = tri
            n = normal(v1, v2, v3)
            f.write(f'  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n')
            f.write('    outer loop\n')
            f.write(f'      vertex {v1[0]:.6f} {v1[1]:.6f} {v1[2]:.6f}\n')
            f.write(f'      vertex {v2[0]:.6f} {v2[1]:.6f} {v2[2]:.6f}\n')
            f.write(f'      vertex {v3[0]:.6f} {v3[1]:.6f} {v3[2]:.6f}\n')
            f.write('    endloop\n')
            f.write('  endfacet\n')
        f.write(f'endsolid {solid_name}\n')


def make_flat_ring(inner_radius: float, outer_radius: float,
                   height: float, segments: int = 64) -> List[Tuple[Vec, Vec, Vec]]:
    """Make a flat annular ring (a hollow cylinder with very small height).

    The ring sits on the XY plane, centred at origin.
    Top face at z=height, bottom face at z=0.

    Returns list of triangles.
    """
    triangles: List[Tuple[Vec, Vec, Vec]] = []

    # Vertices on each circle (segments+1 points but we use mod to wrap)
    def outer(i: int, z: float) -> Vec:
        a = 2.0 * math.pi * i / segments
        return (outer_radius * math.cos(a), outer_radius * math.sin(a), z)

    def inner(i: int, z: float) -> Vec:
        a = 2.0 * math.pi * i / segments
        return (inner_radius * math.cos(a), inner_radius * math.sin(a), z)

    for i in range(segments):
        i_next = (i + 1) % segments

        o_top_i = outer(i, height)
        o_top_n = outer(i_next, height)
        o_bot_i = outer(i, 0.0)
        o_bot_n = outer(i_next, 0.0)
        in_top_i = inner(i, height)
        in_top_n = inner(i_next, height)
        in_bot_i = inner(i, 0.0)
        in_bot_n = inner(i_next, 0.0)

        # --- Top face (annulus, facing +Z) ---
        # Two triangles per segment connecting inner and outer top circles.
        triangles.append((in_top_i, o_top_i, o_top_n))
        triangles.append((in_top_i, o_top_n, in_top_n))

        # --- Bottom face (annulus, facing -Z) — reverse winding ---
        triangles.append((in_bot_i, o_bot_n, o_bot_i))
        triangles.append((in_bot_i, in_bot_n, o_bot_n))

        # --- Outer side (facing outward) ---
        triangles.append((o_bot_i, o_bot_n, o_top_n))
        triangles.append((o_bot_i, o_top_n, o_top_i))

        # --- Inner side (facing inward) ---
        triangles.append((in_bot_i, in_top_n, in_bot_n))
        triangles.append((in_bot_i, in_top_i, in_top_n))

    return triangles


def make_solid_cylinder(radius: float, height: float,
                        segments: int = 32) -> List[Tuple[Vec, Vec, Vec]]:
    """Make a closed vertical cylinder (for the loot beam).

    Cylinder sits on XY plane (bottom at z=0), centred on Z axis.
    """
    triangles: List[Tuple[Vec, Vec, Vec]] = []

    def pt(i: int, z: float) -> Vec:
        a = 2.0 * math.pi * i / segments
        return (radius * math.cos(a), radius * math.sin(a), z)

    centre_bot: Vec = (0.0, 0.0, 0.0)
    centre_top: Vec = (0.0, 0.0, height)

    for i in range(segments):
        i_next = (i + 1) % segments
        a_bot = pt(i, 0.0)
        b_bot = pt(i_next, 0.0)
        a_top = pt(i, height)
        b_top = pt(i_next, height)

        # Side
        triangles.append((a_bot, b_bot, b_top))
        triangles.append((a_bot, b_top, a_top))

        # Bottom cap (facing -Z)
        triangles.append((centre_bot, b_bot, a_bot))

        # Top cap (facing +Z)
        triangles.append((centre_top, a_top, b_top))

    return triangles


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', type=Path, required=True,
                   help='Directory to write STL files into')
    p.add_argument('--segments', type=int, default=64,
                   help='Tessellation segments (more = smoother)')
    args = p.parse_args()

    out_dir: Path = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- selection_ring: small ring under robot_1 ---
    # Husky base ~1m wide; ring should be slightly larger
    ring_tris = make_flat_ring(
        inner_radius=0.65,
        outer_radius=0.80,
        height=0.02,
        segments=args.segments,
    )
    write_stl(ring_tris, 'selection_ring',
              out_dir / 'selection_ring.stl')
    print(f'wrote selection_ring.stl ({len(ring_tris)} triangles)')

    # --- reticle: large ring around tank (radius 8m -> ring at ~6m) ---
    reticle_tris = make_flat_ring(
        inner_radius=5.5,
        outer_radius=6.0,
        height=0.04,
        segments=args.segments,
    )
    write_stl(reticle_tris, 'reticle',
              out_dir / 'reticle.stl')
    print(f'wrote reticle.stl ({len(reticle_tris)} triangles)')

    # --- beam: tall cylinder over tank ---
    # Tank height 6m; beam goes from 6m to 30m for "loot beam reaching skyward"
    beam_tris = make_solid_cylinder(
        radius=0.4,
        height=24.0,
        segments=24,
    )
    write_stl(beam_tris, 'beam',
              out_dir / 'beam.stl')
    print(f'wrote beam.stl ({len(beam_tris)} triangles)')


if __name__ == '__main__':
    main()
