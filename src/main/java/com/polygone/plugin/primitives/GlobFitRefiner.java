package com.polygone.plugin.primitives;

import com.polygone.api.model.GeometricRelationship;
import com.polygone.api.model.PrimitiveFit;
import com.polygone.api.model.PrimitiveType;
import com.polygone.api.model.RelationshipType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Post-processes primitive fits to detect and enforce global geometric relationships
 * (parallel, orthogonal, coplanar, coaxial, equal-radius, concentric).
 *
 * <p>Inspired by the GlobFit paper: detects near-relationships between primitive pairs,
 * scores them by proximity to exact, and snaps the easiest ones first.
 *
 * <p>Pure computational class — no UI or Spring dependencies.
 */
public class GlobFitRefiner {

    private static final Logger log = LoggerFactory.getLogger(GlobFitRefiner.class);

    /**
     * Result of the refinement pass.
     *
     * @param primitives    the (possibly modified) primitive fits
     * @param relationships the detected and enforced geometric relationships
     */
    public record RefinementResult(List<PrimitiveFit> primitives,
                                    List<GeometricRelationship> relationships) {}

    /**
     * Refine primitive fits by detecting and enforcing geometric relationships.
     * Candidate constraints are scored by how close they already are to exact satisfaction,
     * then applied from easiest to hardest while re-validating after each snap so earlier
     * adjustments can influence later ones.
     *
     * @param primitives        list of primitive fits to refine
     * @param angleTolerance    maximum angle deviation (degrees) for parallel/orthogonal detection
     * @param distanceTolerance maximum distance for coplanar/coaxial/concentric detection;
     *                          pass &lt;= 0 to auto-compute (not applicable without bounding box here)
     * @param radiusTolerance   maximum relative radius difference (e.g. 0.05 for 5%)
     * @return refinement result with snapped primitives and detected relationships
     */
    public RefinementResult refine(List<PrimitiveFit> primitives,
                                    double angleTolerance,
                                    double distanceTolerance,
                                    double radiusTolerance) {
        if (primitives == null || primitives.size() < 2) {
            return new RefinementResult(
                    primitives != null ? new ArrayList<>(primitives) : List.of(),
                    List.of());
        }

        double angleToleranceRad = Math.toRadians(angleTolerance);

        // Auto-compute distance tolerance if not provided
        if (distanceTolerance <= 0) {
            distanceTolerance = estimateDistanceTolerance(primitives);
        }

        // Working copy — we'll replace entries as we snap them
        List<PrimitiveFit> working = new ArrayList<>(primitives);

        // Step 1: detect candidate relationships and score them
        List<ScoredRelationship> candidates = new ArrayList<>();
        for (int i = 0; i < working.size(); i++) {
            for (int j = i + 1; j < working.size(); j++) {
                detectRelationships(candidates, working.get(i), i, working.get(j), j,
                        angleToleranceRad, distanceTolerance, radiusTolerance);
            }
        }

        // Step 2: sort by score ascending (easiest / closest to exact first)
        candidates.sort(Comparator.comparingDouble(sr -> sr.score));

        // Step 3: apply each relationship
        List<GeometricRelationship> applied = new ArrayList<>();
        for (ScoredRelationship sr : candidates) {
            PrimitiveFit a = working.get(sr.indexA);
            PrimitiveFit b = working.get(sr.indexB);

            // Re-check validity (parameters may have shifted from earlier snaps)
            if (!isStillValid(sr.type, a, b, angleToleranceRad, distanceTolerance, radiusTolerance)) {
                continue;
            }

            PrimitiveFit[] snapped = applyRelationship(sr.type, a, b);
            if (snapped != null) {
                working.set(sr.indexA, snapped[0]);
                working.set(sr.indexB, snapped[1]);
                applied.add(new GeometricRelationship(sr.type,
                        new int[]{a.sourceRegionId(), b.sourceRegionId()}));
            }
        }

        log.info("GlobFit refinement: {} relationships enforced on {} primitives", applied.size(), working.size());
        return new RefinementResult(working, applied);
    }

    // ────────────────────────────────────────────────────────────────
    // Internal scored relationship record
    // ────────────────────────────────────────────────────────────────

    private record ScoredRelationship(RelationshipType type, int indexA, int indexB, double score) {}

    // ────────────────────────────────────────────────────────────────
    // Relationship detection
    // ────────────────────────────────────────────────────────────────

    private void detectRelationships(List<ScoredRelationship> out,
                                      PrimitiveFit a, int idxA,
                                      PrimitiveFit b, int idxB,
                                      double angleToleranceRad,
                                      double distanceTolerance,
                                      double radiusTolerance) {
        // PARALLEL / ORTHOGONAL / COPLANAR — between planes
        if (a.type() == PrimitiveType.PLANE && b.type() == PrimitiveType.PLANE) {
            double[] na = planeNormal(a);
            double[] nb = planeNormal(b);
            double angle = angleBetweenNormals(na, nb);

            // PARALLEL: angle near 0 or PI
            double parallelDev = Math.min(angle, Math.PI - angle);
            if (parallelDev < angleToleranceRad) {
                double score = parallelDev / angleToleranceRad;
                out.add(new ScoredRelationship(RelationshipType.PARALLEL, idxA, idxB, score));

                // COPLANAR: parallel + close distances
                double da = a.parameters()[3];
                double db = b.parameters()[3];
                // Account for flipped normals
                double effectiveDb = (angle > Math.PI / 2) ? -db : db;
                double distDiff = Math.abs(da - effectiveDb);
                if (distDiff < distanceTolerance) {
                    double coplanarScore = score * 0.5 + (distDiff / distanceTolerance) * 0.5;
                    out.add(new ScoredRelationship(RelationshipType.COPLANAR, idxA, idxB, coplanarScore));
                }
            }

            // ORTHOGONAL: angle near 90°
            double orthoDev = Math.abs(angle - Math.PI / 2);
            if (orthoDev < angleToleranceRad) {
                double score = orthoDev / angleToleranceRad;
                out.add(new ScoredRelationship(RelationshipType.ORTHOGONAL, idxA, idxB, score));
            }
        }

        // PARALLEL / ORTHOGONAL between cylinders (axis directions)
        if (a.type() == PrimitiveType.CYLINDER && b.type() == PrimitiveType.CYLINDER) {
            double[] da = cylinderAxis(a);
            double[] db = cylinderAxis(b);
            double angle = angleBetweenNormals(da, db);

            double parallelDev = Math.min(angle, Math.PI - angle);
            if (parallelDev < angleToleranceRad) {
                out.add(new ScoredRelationship(RelationshipType.PARALLEL, idxA, idxB,
                        parallelDev / angleToleranceRad));
            }

            double orthoDev = Math.abs(angle - Math.PI / 2);
            if (orthoDev < angleToleranceRad) {
                out.add(new ScoredRelationship(RelationshipType.ORTHOGONAL, idxA, idxB,
                        orthoDev / angleToleranceRad));
            }

            // COAXIAL: parallel axes + small offset
            if (parallelDev < angleToleranceRad) {
                double offset = axisOffset(a.parameters(), b.parameters());
                if (offset < distanceTolerance) {
                    double score = (parallelDev / angleToleranceRad) * 0.5
                            + (offset / distanceTolerance) * 0.5;
                    out.add(new ScoredRelationship(RelationshipType.COAXIAL, idxA, idxB, score));
                }
            }
        }

        // EQUAL_RADIUS — cylinders or spheres
        if (hasRadius(a) && hasRadius(b)) {
            double ra = getRadius(a);
            double rb = getRadius(b);
            double maxR = Math.max(ra, rb);
            if (maxR > 1e-12) {
                double relDiff = Math.abs(ra - rb) / maxR;
                if (relDiff < radiusTolerance) {
                    out.add(new ScoredRelationship(RelationshipType.EQUAL_RADIUS, idxA, idxB,
                            relDiff / radiusTolerance));
                }
            }
        }

        // CONCENTRIC — spheres
        if (a.type() == PrimitiveType.SPHERE && b.type() == PrimitiveType.SPHERE) {
            double dx = a.parameters()[0] - b.parameters()[0];
            double dy = a.parameters()[1] - b.parameters()[1];
            double dz = a.parameters()[2] - b.parameters()[2];
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < distanceTolerance) {
                out.add(new ScoredRelationship(RelationshipType.CONCENTRIC, idxA, idxB,
                        dist / Math.max(distanceTolerance, 1e-12)));
            }
        }
    }

    // ────────────────────────────────────────────────────────────────
    // Validity re-check (after earlier snaps may have changed params)
    // ────────────────────────────────────────────────────────────────

    private boolean isStillValid(RelationshipType type, PrimitiveFit a, PrimitiveFit b,
                                  double angleToleranceRad, double distanceTolerance,
                                  double radiusTolerance) {
        return switch (type) {
            case PARALLEL -> {
                double[] na = getDirection(a);
                double[] nb = getDirection(b);
                if (na == null || nb == null) yield false;
                double angle = angleBetweenNormals(na, nb);
                yield Math.min(angle, Math.PI - angle) < angleToleranceRad;
            }
            case ORTHOGONAL -> {
                double[] na = getDirection(a);
                double[] nb = getDirection(b);
                if (na == null || nb == null) yield false;
                double angle = angleBetweenNormals(na, nb);
                yield Math.abs(angle - Math.PI / 2) < angleToleranceRad;
            }
            case COPLANAR -> a.type() == PrimitiveType.PLANE && b.type() == PrimitiveType.PLANE;
            case COAXIAL -> a.type() == PrimitiveType.CYLINDER && b.type() == PrimitiveType.CYLINDER;
            case EQUAL_RADIUS -> hasRadius(a) && hasRadius(b);
            case CONCENTRIC -> a.type() == PrimitiveType.SPHERE && b.type() == PrimitiveType.SPHERE;
        };
    }

    // ────────────────────────────────────────────────────────────────
    // Snap operations
    // ────────────────────────────────────────────────────────────────

    /**
     * Apply a relationship by snapping parameters. Returns [newA, newB] or null if not applicable.
     */
    private PrimitiveFit[] applyRelationship(RelationshipType type, PrimitiveFit a, PrimitiveFit b) {
        return switch (type) {
            case PARALLEL -> snapParallel(a, b);
            case ORTHOGONAL -> snapOrthogonal(a, b);
            case COPLANAR -> snapCoplanar(a, b);
            case COAXIAL -> snapCoaxial(a, b);
            case EQUAL_RADIUS -> snapEqualRadius(a, b);
            case CONCENTRIC -> snapConcentric(a, b);
        };
    }

    private PrimitiveFit[] snapParallel(PrimitiveFit a, PrimitiveFit b) {
        double[] na = getDirection(a);
        double[] nb = getDirection(b);
        if (na == null || nb == null) return null;

        // Check if normals point in same or opposite direction
        double dot = na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2];
        double sign = dot >= 0 ? 1.0 : -1.0;

        // Average direction (accounting for sign)
        double ax = na[0] + sign * nb[0];
        double ay = na[1] + sign * nb[1];
        double az = na[2] + sign * nb[2];
        double len = Math.sqrt(ax * ax + ay * ay + az * az);
        if (len < 1e-12) return null;
        ax /= len;
        ay /= len;
        az /= len;

        PrimitiveFit newA = withDirection(a, ax, ay, az);
        PrimitiveFit newB = withDirection(b, sign * ax, sign * ay, sign * az);
        return new PrimitiveFit[]{newA, newB};
    }

    private PrimitiveFit[] snapOrthogonal(PrimitiveFit a, PrimitiveFit b) {
        double[] na = getDirection(a);
        double[] nb = getDirection(b);
        if (na == null || nb == null) return null;

        // Project nb to be exactly orthogonal to na
        double dot = na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2];
        double ox = nb[0] - dot * na[0];
        double oy = nb[1] - dot * na[1];
        double oz = nb[2] - dot * na[2];
        double len = Math.sqrt(ox * ox + oy * oy + oz * oz);
        if (len < 1e-12) return null;
        ox /= len;
        oy /= len;
        oz /= len;

        PrimitiveFit newB = withDirection(b, ox, oy, oz);
        return new PrimitiveFit[]{a, newB};
    }

    private PrimitiveFit[] snapCoplanar(PrimitiveFit a, PrimitiveFit b) {
        if (a.type() != PrimitiveType.PLANE || b.type() != PrimitiveType.PLANE) return null;

        // First snap parallel
        PrimitiveFit[] par = snapParallel(a, b);
        if (par == null) return null;

        // Then merge distances
        double da = par[0].parameters()[3];
        double db = par[1].parameters()[3];

        // Account for potentially flipped normals
        double[] na = planeNormal(par[0]);
        double[] nb = planeNormal(par[1]);
        double dot = na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2];
        double effectiveDb = dot >= 0 ? db : -db;

        double avgDist = (da + effectiveDb) / 2.0;

        double[] paramsA = par[0].parameters().clone();
        double[] paramsB = par[1].parameters().clone();
        paramsA[3] = avgDist;
        paramsB[3] = dot >= 0 ? avgDist : -avgDist;

        return new PrimitiveFit[]{
                new PrimitiveFit(a.type(), paramsA, a.inlierFaceIndices(), a.fitError(), a.sourceRegionId()),
                new PrimitiveFit(b.type(), paramsB, b.inlierFaceIndices(), b.fitError(), b.sourceRegionId())
        };
    }

    private PrimitiveFit[] snapCoaxial(PrimitiveFit a, PrimitiveFit b) {
        if (a.type() != PrimitiveType.CYLINDER || b.type() != PrimitiveType.CYLINDER) return null;

        double[] pa = a.parameters();
        double[] pb = b.parameters();

        // Average axis direction (accounting for sign)
        double dot = pa[3] * pb[3] + pa[4] * pb[4] + pa[5] * pb[5];
        double sign = dot >= 0 ? 1.0 : -1.0;

        double dx = pa[3] + sign * pb[3];
        double dy = pa[4] + sign * pb[4];
        double dz = pa[5] + sign * pb[5];
        double dlen = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dlen < 1e-12) return null;
        dx /= dlen;
        dy /= dlen;
        dz /= dlen;

        // Average axis point: project both axis points onto the average axis,
        // then find the closest point on the new axis to both original axis lines
        double midX = (pa[0] + pb[0]) / 2.0;
        double midY = (pa[1] + pb[1]) / 2.0;
        double midZ = (pa[2] + pb[2]) / 2.0;

        // Project midpoint onto new axis direction to get a canonical point
        double t = (midX - pa[0]) * dx + (midY - pa[1]) * dy + (midZ - pa[2]) * dz;
        // keep point close to a's original point
        double newPx = pa[0] + t * dx - t * dx; // simplify: use midpoint projected
        // Actually, use the average of the closest points on the new axis to each original point
        double ta = (pa[0] - midX) * dx + (pa[1] - midY) * dy + (pa[2] - midZ) * dz;
        double tb = (pb[0] - midX) * dx + (pb[1] - midY) * dy + (pb[2] - midZ) * dz;

        double[] newParamsA = pa.clone();
        newParamsA[0] = midX;
        newParamsA[1] = midY;
        newParamsA[2] = midZ;
        newParamsA[3] = dx;
        newParamsA[4] = dy;
        newParamsA[5] = dz;

        double[] newParamsB = pb.clone();
        newParamsB[0] = midX;
        newParamsB[1] = midY;
        newParamsB[2] = midZ;
        newParamsB[3] = sign * dx;
        newParamsB[4] = sign * dy;
        newParamsB[5] = sign * dz;

        return new PrimitiveFit[]{
                new PrimitiveFit(a.type(), newParamsA, a.inlierFaceIndices(), a.fitError(), a.sourceRegionId()),
                new PrimitiveFit(b.type(), newParamsB, b.inlierFaceIndices(), b.fitError(), b.sourceRegionId())
        };
    }

    private PrimitiveFit[] snapEqualRadius(PrimitiveFit a, PrimitiveFit b) {
        double ra = getRadius(a);
        double rb = getRadius(b);
        double avg = (ra + rb) / 2.0;

        PrimitiveFit newA = withRadius(a, avg);
        PrimitiveFit newB = withRadius(b, avg);
        if (newA == null || newB == null) return null;
        return new PrimitiveFit[]{newA, newB};
    }

    private PrimitiveFit[] snapConcentric(PrimitiveFit a, PrimitiveFit b) {
        if (a.type() != PrimitiveType.SPHERE || b.type() != PrimitiveType.SPHERE) return null;

        double cx = (a.parameters()[0] + b.parameters()[0]) / 2.0;
        double cy = (a.parameters()[1] + b.parameters()[1]) / 2.0;
        double cz = (a.parameters()[2] + b.parameters()[2]) / 2.0;

        double[] pa = a.parameters().clone();
        double[] pb = b.parameters().clone();
        pa[0] = cx;
        pa[1] = cy;
        pa[2] = cz;
        pb[0] = cx;
        pb[1] = cy;
        pb[2] = cz;

        return new PrimitiveFit[]{
                new PrimitiveFit(a.type(), pa, a.inlierFaceIndices(), a.fitError(), a.sourceRegionId()),
                new PrimitiveFit(b.type(), pb, b.inlierFaceIndices(), b.fitError(), b.sourceRegionId())
        };
    }

    // ────────────────────────────────────────────────────────────────
    // Parameter accessors
    // ────────────────────────────────────────────────────────────────

    private double[] planeNormal(PrimitiveFit pf) {
        double[] p = pf.parameters();
        return new double[]{p[0], p[1], p[2]};
    }

    private double[] cylinderAxis(PrimitiveFit pf) {
        double[] p = pf.parameters();
        return new double[]{p[3], p[4], p[5]};
    }

    /**
     * Get the direction vector for a primitive (normal for planes, axis for cylinders/cones).
     */
    private double[] getDirection(PrimitiveFit pf) {
        return switch (pf.type()) {
            case PLANE -> planeNormal(pf);
            case CYLINDER, CONE -> cylinderAxis(pf);
            default -> null;
        };
    }

    /**
     * Create a new PrimitiveFit with a replaced direction vector.
     */
    private PrimitiveFit withDirection(PrimitiveFit pf, double dx, double dy, double dz) {
        double[] params = pf.parameters().clone();
        switch (pf.type()) {
            case PLANE -> {
                // Recalculate distance: d = n · (d_old/|n_old| * n_old) -> project old plane point
                // Actually: keep the same distance from origin along the new normal direction
                // Old plane: n_old · p = d_old. Pick a point on the old plane and project.
                double oldD = params[3];
                double[] oldN = {params[0], params[1], params[2]};
                // Point on old plane: p = d_old * n_old
                double px = oldD * oldN[0], py = oldD * oldN[1], pz = oldD * oldN[2];
                params[0] = dx;
                params[1] = dy;
                params[2] = dz;
                params[3] = dx * px + dy * py + dz * pz;
            }
            case CYLINDER, CONE -> {
                params[3] = dx;
                params[4] = dy;
                params[5] = dz;
            }
            default -> {
                return pf;
            }
        }
        return new PrimitiveFit(pf.type(), params, pf.inlierFaceIndices(), pf.fitError(), pf.sourceRegionId());
    }

    private boolean hasRadius(PrimitiveFit pf) {
        return pf.type() == PrimitiveType.SPHERE || pf.type() == PrimitiveType.CYLINDER;
    }

    private double getRadius(PrimitiveFit pf) {
        return switch (pf.type()) {
            case SPHERE -> pf.parameters()[3];
            case CYLINDER -> pf.parameters()[6];
            default -> 0;
        };
    }

    private PrimitiveFit withRadius(PrimitiveFit pf, double r) {
        double[] params = pf.parameters().clone();
        switch (pf.type()) {
            case SPHERE -> params[3] = r;
            case CYLINDER -> params[6] = r;
            default -> {
                return null;
            }
        }
        return new PrimitiveFit(pf.type(), params, pf.inlierFaceIndices(), pf.fitError(), pf.sourceRegionId());
    }

    // ────────────────────────────────────────────────────────────────
    // Geometry utilities
    // ────────────────────────────────────────────────────────────────

    private double angleBetweenNormals(double[] a, double[] b) {
        double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        dot = Math.max(-1.0, Math.min(1.0, dot));
        return Math.acos(dot);
    }

    /**
     * Compute the perpendicular distance between two cylinder axes.
     */
    private double axisOffset(double[] pa, double[] pb) {
        // Axis A: point (pa[0..2]), dir (pa[3..5])
        // Axis B: point (pb[0..2]), dir (pb[3..5])
        // Average direction for consistent measurement
        double dx = pa[3], dy = pa[4], dz = pa[5];

        // Vector between axis points
        double wx = pb[0] - pa[0];
        double wy = pb[1] - pa[1];
        double wz = pb[2] - pa[2];

        // Project w onto axis direction and subtract to get perpendicular component
        double dot = wx * dx + wy * dy + wz * dz;
        double perpX = wx - dot * dx;
        double perpY = wy - dot * dy;
        double perpZ = wz - dot * dz;

        return Math.sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);
    }

    /**
     * Estimate a reasonable distance tolerance from the primitive parameters
     * when no bounding box is available.
     */
    private double estimateDistanceTolerance(List<PrimitiveFit> primitives) {
        // Use 1% of the diagonal of the bounding box of all primitive reference points
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, minZ = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE, maxZ = -Double.MAX_VALUE;

        for (PrimitiveFit pf : primitives) {
            double[] p = pf.parameters();
            if (p.length >= 3) {
                minX = Math.min(minX, p[0]);
                minY = Math.min(minY, p[1]);
                minZ = Math.min(minZ, p[2]);
                maxX = Math.max(maxX, p[0]);
                maxY = Math.max(maxY, p[1]);
                maxZ = Math.max(maxZ, p[2]);
            }
        }

        if (minX > maxX) return 0.01; // fallback

        double dx = maxX - minX;
        double dy = maxY - minY;
        double dz = maxZ - minZ;
        double diagonal = Math.sqrt(dx * dx + dy * dy + dz * dz);
        return Math.max(diagonal * 0.01, 1e-6);
    }
}
