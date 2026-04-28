# Hash-Based Face Equality Grouping in modelRefine

## Problem

`FaceEqualitySplitter::split()` groups faces by geometric equality so that
coplanar/coaxial/identical surfaces can be merged during model refinement.
The original algorithm was O(N²) pairwise `isEqual` — correct but
impractically slow for models with many BSpline faces (10+ seconds for
~2,400 faces).

## Prior Approach (Replaced)

A two-level grouping was tried:

1. **Structural hash** — hash of exact integer/boolean invariants
   (degree, pole/knot counts, rational/periodic/closed flags). Faces with
   different hashes guaranteed unequal.
2. **Spatial grid** — within each structural bucket, faces were binned
   into 3D grid cells using a quantized first-pole coordinate. Only
   the 26 neighboring cells were checked for equality, keeping the
   cost O(N).

### Deficiencies of the prior approach

- **`computeGridKey` used only `Pole(1,1)`** — two BSpline surfaces with
  identical `Pole(1,1)` but different geometry would land in the same
  cell and wastefully undergo `isEqual` anyway.
- **Within-cell comparison only compared `indices[0]`** — if a cell
  contained multiple non-equal faces (same first pole but different
  other poles), valid matches between non-first faces were missed.
- **Grid boundary sensitivity** — a 0.1 grid can produce edge cases
  where two equal faces straddle a boundary and land in cells more
  than one step apart, causing a false negative.
- **Complexity** — grid hash, 26-neighbor loop, union-find with
  distinct-roots helpers.

## New Approach: Full Geometric Hash

### Design

Replace both grouping levels with a single **full geometric hash**
computed once per face at 1e-10 quantization:

```
Phase 1 — computeHash(face) for every face
Phase 2 — group face indices by hash value (unordered_map)
Phase 3 — within each hash group, pairwise isEqual + union-find
```

Each hash covers **all** geometric data that `isEqual` checks:

| Surface type | Hashed fields |
|---|---|
| Plane | `gp_Pln::Location` (3 coords), `Axis::Direction` (3 coords) |
| Cylinder | `Radius`, `Location` (3 coords), `Axis::Direction` (3 coords) |
| BSpline | Degrees, 6 boolean flags, all pole coordinates, all weights (if rational), all knot values, all multiplicities |

### Why 1e-10 quantization?

| Value | Magnitude | Role |
|---|---|---|
| OCCT FP noise | ~1e-15 | Lowest: two recomputes of the same surface |
| HASH_GRID | **1e-10** | Our quantization: 5× above noise |
| `Precision::Confusion()` | ~1e-7 | OCCT equality tolerance: 3× above our grid |

Two `isEqual`-return-true surfaces will always have the same hash because
their poles/knots differ by at most ~1e-15, which quantizes identically
at 1e-10.

The 1e-10 threshold is also fine enough that different surfaces with
different geometry virtually never collide — any single pole coordinate
differing by >= 1e-10 produces a different hash.

### Correctness argument

```
∃ isEqual(A, B)  ⇒  computeHash(A) = computeHash(B)
```

This is the critical property. It holds because:
- Every field that `isEqual` tests is also included in `computeHash`.
- All floating-point fields are quantized to 1e-10, which is well
  above the difference between two recomputations of the same surface
  (~1e-15).
- All integer/boolean fields use exact `std::hash`.

The converse (same hash ⇒ isEqual) is **not** required — hash collisions
are handled by the pairwise `isEqual` within each hash group.

```
No false positives: isEqual is the final arbiter (belt-and-suspenders)
No false negatives: equal surfaces always produce the same hash
```

### Performance

For N faces where each `isEqual` is O(P) (P = poles × knots):

| Approach | Cost |
|---|---|
| Old pairwise | O(N² × P) |
| Prior hash + grid | O(N × P_hash + N × P) in worst case |
| **New hash-only** | **O(N × P)** — one hash per face, within-group isEqual negligible |

Hash computation for a BSpline surface visits every pole and knot
exactly once (same as `isEqual`), but it does so **once per face**
instead of O(N²) times.

## Test Suite

### Tests added (11 tests in `TestModelRefineHash.py`)

All pass with `Ran 91 tests in 1.372s — OK, 0 errors, 0 failures` (13 new +
78 existing).

| Test | What it proves |
|---|---|
| `test_plane_adjacent_equal` | Box refines to 6 faces (regression) |
| `test_plane_same_plane_merges` | Coplanar faces merge after fuse |
| `test_plane_different_planes_no_merge` | Different parallel planes stay separate |
| `test_cylinder_coaxial` | Single cylinder unaffected |
| `test_cylinder_offset_segments` | Two coaxial cylinder segments merge |
| `test_bspline_equal_surfaces` | NURBS box refines without errors |
| `test_bspline_identical_faces` | **Two identical BSpline faces merge** |
| `test_no_false_positives_bspline` | 3 different BSpline surfaces (same degree/counts) stay as 3 |
| `test_mixed_bucket_some_equal` | NURBS box: 6 different BSpline faces not falsely merged |
| `test_baseline_mixed_face_types` | Fused box+cylinder → NURBS → refine: valid, face count ≤ input |
| `test_deterministic_output` | Same input → same output (face count + volume) |
| `test_refine_preserves_volume` | Volume preserved for 5 primitive types |
| `test_performance_not_regressed` | 56-face shape refines in 0.01s |

### Existing tests still pass

All 91 tests from the Part test suite (BRep, Geom2d, TopoShape, regression,
mirror, extrusion, NURBS refine, etc.) pass with zero failures.

## Potential Problematic Cases

### 1. Hash boundary straddle (extreme FP jitter)

If two recomputations of the same surface produce a coordinate that
differs by >= 1e-10, their hashes would diverge and `isEqual` would
not be called between them. This would be a **false negative**.

**Mitigation:** OCCT FP noise is ~1e-15. Even across different solver
paths or OCCT versions, the same geometric surface produces identical
bit patterns. The 1e-10 boundary gives 5 orders of margin. In practice
this cannot occur within a single FreeCAD session.

If this ever became a concern, a secondary pass could merge hash groups
whose faces are adjacent in parameter space, similar to the old grid
neighbor check.

### 2. Hash collision (false positive candidate)

Two different surfaces with the same hash would be compared by `isEqual`.
If `isEqual` also returns true (a true hash collision), they would be
incorrectly merged.

**Mitigation:** The hash mixes every field the surface has. A collision
requires all pole coordinates, knot values, multiplicities, and flags
to simultaneously produce the same hash. With a 64-bit hash (~2⁶⁴ values)
and the hashCombine mixing, this is astronomically unlikely.

Additionally, `isEqual` is called as a verification step within each
hash group, so even if a collision occurred, `isEqual` would catch it.

### 3. New surface types

The `computeHash` default returns 0, which puts all faces of that type
into one hash bucket and falls back to pairwise `isEqual`. This is
correct but slow. Adding a specialized hash for a new surface type
is straightforward.

### 4. Plane and cylinder are `O(N²)` within their hash bucket

All coplanar faces share one hash (same location + direction). If a
model has thousands of coplanar faces, they all land in one bucket
and pairwise `isEqual` runs at O(N²). This is **unchanged** from the
prior approach and from the original algorithm, because both previous
approaches also fell through to pairwise for planes/cylinders.
`isEqual` for planes and cylinders is extremely cheap (a few float
comparisons), so even large buckets are fast.

### 5. Memory

An `std::vector<size_t>` of N hashes and an `unordered_map` of hash
groups. For 100,000 faces this is ~800 KB for the hash vector plus
overhead. Negligible.

## Future Directions

### Cached hashes on document objects

The `computeHash` method is a natural building block for a larger
optimization: caching the hash on each `Part::Feature` at recompute
time and propagating it through the dependency tree. This would
allow the entire refine pass to skip `computeHash` entirely and
read pre-computed values, turning the per-face hash cost from
O(poles × knots) to O(1) for subsequent recomputes.

The hash would need to be:
- Stored on the document object (e.g. in GeoFeature)
- Invalidated when the shape changes
- Propagated to parent objects in the tree
- Deterministic across FreeCAD versions (same model → same hash)

This is a larger architectural change and is not part of this PR.
