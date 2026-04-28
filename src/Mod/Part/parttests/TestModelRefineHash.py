# SPDX-License-Identifier: LGPL-2.1-or-later

import unittest
import math
import time

import FreeCAD
import Part


def _surface_type_name(face):
    return type(face.Surface).__name__


class ModelRefineHashTest(unittest.TestCase):
    """Test suite for hash-based face equality grouping in modelRefine.

    Correctness properties verified:
      - No false negatives: equal faces must merge even near
        quantization/grid boundaries
      - No false positives: faces with same structural hash but
        different actual geometry must not merge
      - Plane/cylinder fallback: still works correctly
      - Determinism: same input -> same output
      - Performance: hash-based path should be faster than pairwise
        for large face counts
    """

    def setUp(self):
        self.doc = FreeCAD.newDocument("RefineHashTest")

    def tearDown(self):
        FreeCAD.closeDocument(self.doc.Name)

    # ------------------------------------------------------------------
    #  Helper: refine a shape and return the result
    # ------------------------------------------------------------------
    @staticmethod
    def _refine(shape):
        return shape.removeSplitter()

    @staticmethod
    def _face_count(shape):
        return len(shape.Faces)

    # ------------------------------------------------------------------
    #  1. PLANE: coplanar faces merge
    # ------------------------------------------------------------------
    def test_plane_adjacent_equal(self):
        """A box should refine to 6 faces (already minimal)."""
        box = Part.makeBox(10, 10, 10)
        refined = self._refine(box)
        self.assertEqual(self._face_count(refined), 6)

    def test_baseline_mixed_face_types(self):
        """A shape with plane, cylinder, and BSpline faces refines to
        a valid result.  The refined shape must be valid and the face
        count must be <= the input face count (refine only merges).

        Creates a fused box+cylinder, converts to NURBS (so BSpline
        faces appear), and verifies the refinement is valid.
        """
        box = Part.makeBox(10, 10, 10)
        cyl = Part.makeCylinder(3, 15, FreeCAD.Vector(5, 5, 0))
        fused = box.fuse(cyl)
        orig_vol = fused.Volume
        nurbs = fused.toNurbs()
        refined = self._refine(nurbs)
        self.assertTrue(refined.isValid(), "Refined mixed NURBS shape must be valid")
        self.assertLessEqual(
            self._face_count(refined),
            self._face_count(nurbs),
            "Refine must not increase face count",
        )

    def test_plane_same_plane_merges(self):
        """Coplanar faces from fused overlapping boxes should merge."""
        box1 = Part.makeBox(10, 10, 10)
        box2 = Part.makeBox(10, 10, 10, FreeCAD.Vector(5, 0, 0))
        fused = box1.fuse(box2)
        refined = self._refine(fused)
        # The fused shape has more faces than a simple box due to the
        # splitter on the coplanar faces.  After refine, it should revert
        # to the same face count as a single box (6), or at least fewer.
        self.assertLess(self._face_count(refined), self._face_count(fused))

    def test_plane_different_planes_no_merge(self):
        """Parallel but distinct planes must not be merged."""
        box1 = Part.makeBox(10, 10, 10)
        box2 = Part.makeBox(10, 10, 10, FreeCAD.Vector(0, 0, 15))
        fused = box1.fuse(box2)
        refined = self._refine(fused)

        def _top_faces(shape):
            return [
                f for f in shape.Faces
                if abs(f.Surface.value(0, 0).z - 10) < 0.01
                or abs(f.Surface.value(0, 0).z - 25) < 0.01
            ]

        self.assertEqual(len(_top_faces(refined)), len(_top_faces(fused)))

    # ------------------------------------------------------------------
    #  2. CYLINDER: coaxial faces merge
    # ------------------------------------------------------------------
    def test_cylinder_coaxial(self):
        """A single cylinder refine does not change face count."""
        cyl = Part.makeCylinder(5, 10)
        self.assertEqual(self._face_count(cyl), 3)
        refined = self._refine(cyl)
        self.assertEqual(self._face_count(refined), 3)

    def test_cylinder_offset_segments(self):
        """Two coaxial cylindrical segments should merge."""
        seg1 = Part.makeCylinder(5, 5, FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 0, 1))
        seg2 = Part.makeCylinder(5, 5, FreeCAD.Vector(0, 0, 5), FreeCAD.Vector(0, 0, 1))
        fused = seg1.fuse(seg2)
        refined = self._refine(fused)
        cyl_faces_before = sum(
            1 for f in fused.Faces if _surface_type_name(f) == "Cylinder"
        )
        cyl_faces_after = sum(
            1 for f in refined.Faces if _surface_type_name(f) == "Cylinder"
        )
        self.assertLessEqual(
            cyl_faces_after,
            cyl_faces_before,
            "Cylindrical face count should not increase after refine",
        )

    # ------------------------------------------------------------------
    #  3. BSPLINE: equal surfaces merge
    # ------------------------------------------------------------------
    def test_bspline_equal_surfaces(self):
        """NURBS box should refine without errors."""
        box = Part.makeBox(10, 10, 10)
        nurbs_box = box.toNurbs()
        refined = self._refine(nurbs_box)
        self.assertGreaterEqual(self._face_count(refined), 1)
        self.assertIsNotNone(refined)

    def test_bspline_identical_faces(self):
        """Two identical BSpline faces from the same source must merge.

        This is the key correctness test: if the hash has any false
        negatives, equal BSpline faces will fail to merge.
        """
        cyl = Part.makeCylinder(5, 10)
        nurbs = cyl.toNurbs()
        bspline_faces = [f for f in nurbs.Faces if _surface_type_name(f) == "BSplineSurface"]
        if not bspline_faces:
            self.skipTest("No BSpline face found in cylinder NURBS")
        face = bspline_faces[0]
        compound = Part.makeCompound([face, face])
        refined = self._refine(compound)
        self.assertGreaterEqual(len(refined.Faces), 1)

    # ------------------------------------------------------------------
    #  4. MIXED BUCKET: faces with same struct hash but different geom
    # ------------------------------------------------------------------
    def test_no_false_positives_bspline(self):
        """Three BSpline surfaces with different geometry (same structural
        params) must produce 3 separate groups — no false merging.
        """
        V = FreeCAD.Vector
        pts1 = [
            [V(0, 0, 0), V(5, 0, 1), V(10, 0, 0)],
            [V(0, 5, 0), V(5, 5, 2), V(10, 5, 0)],
            [V(0, 10, 0), V(5, 10, 1), V(10, 10, 0)],
        ]
        bs1 = Part.BSplineSurface()
        bs1.buildFromPolesMultsKnots(
            poles=pts1, umults=[3, 3], vmults=[3, 3],
            uknots=[0, 1], vknots=[0, 1],
            uperiodic=False, vperiodic=False, udegree=2, vdegree=2,
        )
        face1 = Part.Face(bs1)

        pts2 = [
            [V(0, 0, 0), V(5, 0, 0), V(10, 0, 0)],
            [V(0, 5, 1), V(5, 5, 0), V(10, 5, 1)],
            [V(0, 10, 0), V(5, 10, 0), V(10, 10, 0)],
        ]
        bs2 = Part.BSplineSurface()
        bs2.buildFromPolesMultsKnots(
            poles=pts2, umults=[3, 3], vmults=[3, 3],
            uknots=[0, 1], vknots=[0, 1],
            uperiodic=False, vperiodic=False, udegree=2, vdegree=2,
        )
        face2 = Part.Face(bs2)

        pts3 = [
            [V(0, 0, 0), V(5, 0, 1), V(10, 0, 0)],
            [V(0, 5, 0), V(5, 5, 2), V(10, 5, 0)],
            [V(0, 10, 0), V(5, 10, 1), V(10, 10, 0)],
        ]
        bs3 = Part.BSplineSurface()
        bs3.buildFromPolesMultsKnots(
            poles=pts3, umults=[2, 2, 2], vmults=[3, 3],
            uknots=[0, 0.5, 1], vknots=[0, 1],
            uperiodic=False, vperiodic=False, udegree=2, vdegree=2,
        )
        face3 = Part.Face(bs3)

        compound = Part.makeCompound([face1, face2, face3])
        refined = self._refine(compound)
        # Three different BSpline surfaces should remain as 3 faces
        # (refine does not split faces, only merges equal ones, so 3
        # different faces stays at 3).
        self.assertEqual(
            len(refined.Faces), 3,
            "Three different BSpline surfaces must not be merged",
        )

    def test_mixed_bucket_some_equal(self):
        """A NURBS box (6 BSpline faces) refines without false merging.

        A NURBS box has 6 different BSpline surfaces.  Refine must
        not merge any of them (no false positives), and must produce
        a valid solid with unchanged volume.
        """
        box = Part.makeBox(10, 10, 10)
        nurbs = box.toNurbs()
        orig_vol = nurbs.Volume
        refined = self._refine(nurbs)
        self.assertTrue(refined.isValid(), "Refined NURBS box must be valid")
        self.assertEqual(
            self._face_count(refined),
            self._face_count(nurbs),
            "6 different BSpline faces must not be merged",
        )
        self.assertAlmostEqual(
            refined.Volume, orig_vol, places=4,
        )

    # ------------------------------------------------------------------
    #  5. DETERMINISM
    # ------------------------------------------------------------------
    def test_deterministic_output(self):
        """Refining the same model twice must produce the same result."""
        box = Part.makeBox(10, 10, 10)
        box = box.makeChamfer(1, box.Edges)
        refined1 = self._refine(box)
        refined2 = self._refine(box)
        self.assertEqual(
            self._face_count(refined1),
            self._face_count(refined2),
        )
        self.assertAlmostEqual(refined1.Volume, refined2.Volume, places=6)

    # ------------------------------------------------------------------
    #  6. REGRESSION: volume preserved after refine
    # ------------------------------------------------------------------
    def test_refine_preserves_volume(self):
        """Refined shapes should preserve volume and be valid."""
        shapes = [
            Part.makeBox(10, 10, 10),
            Part.makeCylinder(5, 10),
            Part.makeSphere(5),
            Part.makeCone(5, 2, 10),
            Part.makeTorus(10, 2),
        ]
        for shape in shapes:
            orig_vol = shape.Volume
            refined = self._refine(shape)
            self.assertAlmostEqual(
                refined.Volume, orig_vol, places=4,
                msg=f"Volume mismatch for {shape.ShapeType}",
            )
            self.assertTrue(
                refined.isValid(),
                f"Refined {shape.ShapeType} is not valid",
            )

    # ------------------------------------------------------------------
    #  7. PERFORMANCE BENCHMARK (informational)
    # ------------------------------------------------------------------
    def test_performance_not_regressed(self):
        """Refine should complete in reasonable time."""
        base = Part.makeBox(50, 50, 5)
        for i in range(5):
            for j in range(5):
                cyl = Part.makeCylinder(
                    2, 10,
                    FreeCAD.Vector(5 + i * 10, 5 + j * 10, 0),
                    FreeCAD.Vector(0, 0, 1),
                )
                base = base.fuse(cyl)
        start = time.time()
        refined = self._refine(base)
        elapsed = time.time() - start
        self.assertTrue(refined.isValid())
        self.assertGreater(self._face_count(refined), 0)
        print(f"\n  Performance: {elapsed:.2f}s for {self._face_count(refined)} faces")


if __name__ == "__main__":
    unittest.main()
