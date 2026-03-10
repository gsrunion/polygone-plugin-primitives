package com.polygone.plugin.primitives;

import com.polygone.api.model.FaceRegion;
import com.polygone.api.model.MeshData;
import com.polygone.api.model.PrimitiveFit;
import com.polygone.api.model.PrimitiveType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

/**
 * RANSAC-based primitive fitting for face regions.
 *
 * <p>For each region, tries fitting PLANE, SPHERE, CYLINDER, and CONE primitives
 * using random sampling, then picks the type with the best inlier count (ties
 * broken by lower RMS error). Regions with insufficient inliers are labeled FREEFORM.
 *
 * <p>Pure computational class — no UI or Spring dependencies.
 *
 * <p>Parameter layouts:
 * PLANE:    [nx, ny, nz, d]
 * SPHERE:   [cx, cy, cz, r]
 * CYLINDER: [px, py, pz, dx, dy, dz, r, height]
 * CONE:     [ax, ay, az, dx, dy, dz, halfAngle, height]
 */
public class RansacPrimitiveFitter {

    private static final Logger log = LoggerFactory.getLogger(RansacPrimitiveFitter.class);

    private static final long DEFAULT_SEED = 42L;

    /**
     * Fit primitives to all regions.
     *
     * @param mesh              the source mesh
     * @param regions           face regions to fit
     * @param maxIterations     RANSAC iterations per primitive type
     * @param distanceThreshold maximum distance from primitive surface to count as inlier
     * @param minInlierRatio    minimum fraction of inlier faces to accept a fit (otherwise FREEFORM)
     * @return list of primitive fits, one per region
     */
    public List<PrimitiveFit> fitAll(MeshData mesh, List<FaceRegion> regions,
                                      int maxIterations, double distanceThreshold,
                                      double minInlierRatio, double maxFitError) {
        List<PrimitiveFit> results = new ArrayList<>(regions.size());
        for (FaceRegion region : regions) {
            results.add(fitRegion(mesh, region, null, maxIterations, distanceThreshold, minInlierRatio, maxFitError));
        }
        return results;
    }

    /**
     * Fit a single region, optionally forcing a specific primitive type.
     *
     * @param mesh              the source mesh
     * @param region            the face region to fit
     * @param forceType         if non-null, only try this primitive type
     * @param maxIterations     RANSAC iterations per primitive type
     * @param distanceThreshold maximum distance from primitive surface to count as inlier
     * @param minInlierRatio    minimum fraction of inlier faces to accept a fit
     * @return the best primitive fit for this region
     */
    public PrimitiveFit fitRegion(MeshData mesh, FaceRegion region, PrimitiveType forceType,
                                   int maxIterations, double distanceThreshold,
                                   double minInlierRatio, double maxFitError) {
        float[] vertices = mesh.getVertices();
        int[] meshFaces = mesh.getFaces();
        int[] faceIndices = region.faceIndices();
        int n = faceIndices.length;

        // Extract face centroids and normals for this region
        double[][] centroids = new double[n][3];
        double[][] normals = new double[n][3];
        for (int i = 0; i < n; i++) {
            int f = faceIndices[i];
            int base = f * 6;
            int vi0 = meshFaces[base] * 3;
            int vi1 = meshFaces[base + 2] * 3;
            int vi2 = meshFaces[base + 4] * 3;

            centroids[i][0] = (vertices[vi0] + vertices[vi1] + vertices[vi2]) / 3.0;
            centroids[i][1] = (vertices[vi0 + 1] + vertices[vi1 + 1] + vertices[vi2 + 1]) / 3.0;
            centroids[i][2] = (vertices[vi0 + 2] + vertices[vi1 + 2] + vertices[vi2 + 2]) / 3.0;

            double e1x = vertices[vi1] - vertices[vi0];
            double e1y = vertices[vi1 + 1] - vertices[vi0 + 1];
            double e1z = vertices[vi1 + 2] - vertices[vi0 + 2];
            double e2x = vertices[vi2] - vertices[vi0];
            double e2y = vertices[vi2 + 1] - vertices[vi0 + 1];
            double e2z = vertices[vi2 + 2] - vertices[vi0 + 2];

            double nx = e1y * e2z - e1z * e2y;
            double ny = e1z * e2x - e1x * e2z;
            double nz = e1x * e2y - e1y * e2x;
            double len = Math.sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 1e-12) {
                normals[i][0] = nx / len;
                normals[i][1] = ny / len;
                normals[i][2] = nz / len;
            }
        }

        PrimitiveType[] typesToTry;
        if (forceType != null) {
            typesToTry = new PrimitiveType[]{forceType};
        } else {
            typesToTry = new PrimitiveType[]{
                    PrimitiveType.PLANE, PrimitiveType.SPHERE,
                    PrimitiveType.CYLINDER, PrimitiveType.CONE
            };
        }

        // Compute bounding box of region for plausibility checks
        double[] bboxMin = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
        double[] bboxMax = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (int i = 0; i < n; i++) {
            for (int d = 0; d < 3; d++) {
                bboxMin[d] = Math.min(bboxMin[d], centroids[i][d]);
                bboxMax[d] = Math.max(bboxMax[d], centroids[i][d]);
            }
        }
        double bboxDiag = Math.sqrt(
                (bboxMax[0] - bboxMin[0]) * (bboxMax[0] - bboxMin[0])
                + (bboxMax[1] - bboxMin[1]) * (bboxMax[1] - bboxMin[1])
                + (bboxMax[2] - bboxMin[2]) * (bboxMax[2] - bboxMin[2]));

        Random rng = new Random(DEFAULT_SEED);
        PrimitiveFit bestFit = null;
        double bestScore = -Double.MAX_VALUE;
        double bestRms = Double.MAX_VALUE;

        for (PrimitiveType type : typesToTry) {
            int minSamples = minSamplesFor(type);
            if (n < minSamples) {
                continue;
            }

            double[] bestParams = null;
            int bestIterInliers = -1;
            double bestIterRms = Double.MAX_VALUE;

            for (int iter = 0; iter < maxIterations; iter++) {
                int[] sampleIdx = randomSample(rng, n, minSamples);
                double[] params = fitCandidate(type, centroids, normals, sampleIdx);
                if (params == null) {
                    continue;
                }

                // Geometric plausibility check — reject degenerate fits early
                if (!isGeometricallyPlausible(type, params, bboxDiag)) {
                    continue;
                }

                // Count inliers
                int inlierCount = 0;
                double sumSqDist = 0;
                for (int i = 0; i < n; i++) {
                    double dist = distanceTo(type, params, centroids[i]);
                    if (dist <= distanceThreshold) {
                        inlierCount++;
                        sumSqDist += dist * dist;
                    }
                }

                if (inlierCount > bestIterInliers ||
                        (inlierCount == bestIterInliers && inlierCount > 0
                                && Math.sqrt(sumSqDist / inlierCount) < bestIterRms)) {
                    bestIterInliers = inlierCount;
                    bestIterRms = inlierCount > 0 ? Math.sqrt(sumSqDist / inlierCount) : Double.MAX_VALUE;
                    bestParams = params;
                }
            }

            if (bestParams == null || bestIterInliers <= 0) {
                continue;
            }

            // Normal consistency check — reject fits where face normals disagree
            // with the expected orientation for the fitted primitive.
            // Planes require higher consistency since they're the simplest model.
            double normalConsistency = computeNormalConsistency(type, bestParams, centroids, normals, n, distanceThreshold);
            double minConsistency = (type == PrimitiveType.PLANE) ? 0.8 : 0.5;
            if (normalConsistency < minConsistency) {
                log.debug("Region {}: rejecting {} (normal consistency {} < {})",
                        region.regionId(), type, String.format("%.2f", normalConsistency), String.format("%.2f", minConsistency));
                continue;
            }

            // Angular coverage check for cylinders: require faces to wrap
            // most of the way around the axis (no partial arcs for now).
            if (type == PrimitiveType.CYLINDER) {
                double angularCoverage = computeCylinderAngularCoverage(
                        bestParams, centroids, n, distanceThreshold);
                if (angularCoverage < 270.0) {
                    log.debug("Region {}: rejecting CYLINDER (angular coverage {}° < 270°)",
                            region.regionId(), String.format("%.0f", angularCoverage));
                    continue;
                }
            }

            // Complexity penalty: prefer simpler primitives (Occam's razor).
            // PLANE has fewest params, then SPHERE, CYLINDER, CONE.
            // Penalty is a fraction of region size so it scales with data.
            double complexityPenalty = complexityPenaltyFor(type, n);
            double score = bestIterInliers - complexityPenalty;

            if (score > bestScore || (Math.abs(score - bestScore) < 0.5 && bestIterRms < bestRms)) {
                bestScore = score;
                bestRms = bestIterRms;

                // Collect inlier face indices (global indices)
                List<Integer> inlierList = new ArrayList<>();
                for (int i = 0; i < n; i++) {
                    double dist = distanceTo(type, bestParams, centroids[i]);
                    if (dist <= distanceThreshold) {
                        inlierList.add(faceIndices[i]);
                    }
                }

                bestFit = new PrimitiveFit(type, bestParams,
                        inlierList.stream().mapToInt(Integer::intValue).toArray(),
                        bestIterRms, region.regionId());
            }
        }

        // Check inlier ratio and fit error threshold
        int bestInlierCount = bestFit != null ? bestFit.inlierFaceIndices().length : 0;
        if (bestFit != null && (double) bestInlierCount / n >= minInlierRatio) {
            if (maxFitError > 0 && bestRms > maxFitError) {
                log.debug("Region {}: FREEFORM ({} RMS={} exceeds maxFitError={})",
                        region.regionId(), bestFit.type(), String.format("%.6f", bestRms), maxFitError);
            } else {
                log.debug("Region {}: {} ({}/{} inliers, RMS={}, score={})",
                        region.regionId(), bestFit.type(), bestInlierCount, n,
                        String.format("%.6f", bestRms), String.format("%.1f", bestScore));
                return bestFit;
            }
        } else {
            // Fall back to FREEFORM
            log.debug("Region {}: FREEFORM (best inlier ratio {} < {})",
                    region.regionId(),
                    n > 0 && bestInlierCount >= 0 ? String.format("%.2f", (double) bestInlierCount / n) : "N/A",
                    minInlierRatio);
        }
        return new PrimitiveFit(PrimitiveType.FREEFORM, new double[0], faceIndices, Double.NaN, region.regionId());
    }

    // ────────────────────────────────────────────────────────────────
    // Minimum sample sizes
    // ────────────────────────────────────────────────────────────────

    private int minSamplesFor(PrimitiveType type) {
        // Minimum face count per region to attempt fitting this type.
        // Higher values prevent false positives on tiny arc segments.
        return switch (type) {
            case PLANE -> 3;
            case SPHERE -> 8;
            case CYLINDER -> 12;
            case CONE -> 15;
            case FREEFORM -> Integer.MAX_VALUE;
        };
    }

    // ────────────────────────────────────────────────────────────────
    // Random sampling
    // ────────────────────────────────────────────────────────────────

    private int[] randomSample(Random rng, int population, int sampleSize) {
        int[] sample = new int[sampleSize];
        for (int i = 0; i < sampleSize; i++) {
            boolean unique;
            do {
                sample[i] = rng.nextInt(population);
                unique = true;
                for (int j = 0; j < i; j++) {
                    if (sample[j] == sample[i]) {
                        unique = false;
                        break;
                    }
                }
            } while (!unique);
        }
        return sample;
    }

    // ────────────────────────────────────────────────────────────────
    // Candidate fitting
    // ────────────────────────────────────────────────────────────────

    private double[] fitCandidate(PrimitiveType type, double[][] centroids,
                                   double[][] normals, int[] sampleIdx) {
        return switch (type) {
            case PLANE -> fitPlane(centroids, sampleIdx);
            case SPHERE -> fitSphere(centroids, sampleIdx);
            case CYLINDER -> fitCylinder(centroids, normals, sampleIdx);
            case CONE -> fitCone(centroids, normals, sampleIdx);
            case FREEFORM -> null;
        };
    }

    // ────────────────────────── PLANE ──────────────────────────

    /**
     * Fit plane from 3 sample centroids via cross product.
     * Parameters: [nx, ny, nz, distance]
     */
    private double[] fitPlane(double[][] centroids, int[] idx) {
        double[] p0 = centroids[idx[0]];
        double[] p1 = centroids[idx[1]];
        double[] p2 = centroids[idx[2]];

        double e1x = p1[0] - p0[0], e1y = p1[1] - p0[1], e1z = p1[2] - p0[2];
        double e2x = p2[0] - p0[0], e2y = p2[1] - p0[1], e2z = p2[2] - p0[2];

        double nx = e1y * e2z - e1z * e2y;
        double ny = e1z * e2x - e1x * e2z;
        double nz = e1x * e2y - e1y * e2x;

        double len = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-12) return null;

        nx /= len;
        ny /= len;
        nz /= len;
        double d = nx * p0[0] + ny * p0[1] + nz * p0[2];
        return new double[]{nx, ny, nz, d};
    }

    // ────────────────────────── SPHERE ──────────────────────────

    /**
     * Fit sphere from 4 sample centroids using Cramer's rule.
     * |p - c|^2 = r^2; subtracting pairs yields 3 linear equations.
     * Parameters: [cx, cy, cz, radius]
     */
    private double[] fitSphere(double[][] centroids, int[] idx) {
        double[] p0 = centroids[idx[0]];
        double[] p1 = centroids[idx[1]];
        double[] p2 = centroids[idx[2]];
        double[] p3 = centroids[idx[3]];

        // Subtract p0 equation from p1, p2, p3 to get 3 linear equations:
        // 2*(p1-p0)·c = |p1|^2 - |p0|^2  etc.
        double[][] A = new double[3][3];
        double[] b = new double[3];
        double sq0 = p0[0] * p0[0] + p0[1] * p0[1] + p0[2] * p0[2];

        for (int i = 0; i < 3; i++) {
            double[] pi = centroids[idx[i + 1]];
            A[i][0] = 2 * (pi[0] - p0[0]);
            A[i][1] = 2 * (pi[1] - p0[1]);
            A[i][2] = 2 * (pi[2] - p0[2]);
            b[i] = (pi[0] * pi[0] + pi[1] * pi[1] + pi[2] * pi[2]) - sq0;
        }

        double[] c = solve3x3(A, b);
        if (c == null) return null;

        double r = Math.sqrt((c[0] - p0[0]) * (c[0] - p0[0])
                + (c[1] - p0[1]) * (c[1] - p0[1])
                + (c[2] - p0[2]) * (c[2] - p0[2]));

        if (r < 1e-12 || !Double.isFinite(r)) return null;
        return new double[]{c[0], c[1], c[2], r};
    }

    // ────────────────────────── CYLINDER ──────────────────────────

    /**
     * Fit cylinder using face normals and centroids.
     * Axis direction estimated from cross products of normal pairs.
     * Then project centroids perpendicular to axis and fit a 2D circle.
     * Parameters: [px, py, pz, dx, dy, dz, radius, height]
     */
    private double[] fitCylinder(double[][] centroids, double[][] normals, int[] idx) {
        // Estimate axis from cross products of normal pairs
        double axisX = 0, axisY = 0, axisZ = 0;
        int crossCount = 0;
        for (int i = 0; i < idx.length; i++) {
            for (int j = i + 1; j < idx.length; j++) {
                double[] ni = normals[idx[i]];
                double[] nj = normals[idx[j]];
                double cx = ni[1] * nj[2] - ni[2] * nj[1];
                double cy = ni[2] * nj[0] - ni[0] * nj[2];
                double cz = ni[0] * nj[1] - ni[1] * nj[0];
                double clen = Math.sqrt(cx * cx + cy * cy + cz * cz);
                if (clen > 1e-8) {
                    // Normalize and accumulate (ensure consistent direction)
                    cx /= clen;
                    cy /= clen;
                    cz /= clen;
                    if (crossCount > 0) {
                        double dot = cx * axisX + cy * axisY + cz * axisZ;
                        if (dot < 0) {
                            cx = -cx;
                            cy = -cy;
                            cz = -cz;
                        }
                    }
                    axisX += cx;
                    axisY += cy;
                    axisZ += cz;
                    crossCount++;
                }
            }
        }

        if (crossCount == 0) return null;

        double alen = Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
        if (alen < 1e-12) return null;
        axisX /= alen;
        axisY /= alen;
        axisZ /= alen;

        // Compute centroid of sample points
        double meanX = 0, meanY = 0, meanZ = 0;
        for (int i : idx) {
            meanX += centroids[i][0];
            meanY += centroids[i][1];
            meanZ += centroids[i][2];
        }
        meanX /= idx.length;
        meanY /= idx.length;
        meanZ /= idx.length;

        // Build local 2D coordinate system perpendicular to axis
        double[] u = perpendicular(axisX, axisY, axisZ);
        double vx = axisY * u[2] - axisZ * u[1];
        double vy = axisZ * u[0] - axisX * u[2];
        double vz = axisX * u[1] - axisY * u[0];

        // Project sample points onto 2D (u, v) plane
        double[] px2d = new double[idx.length];
        double[] py2d = new double[idx.length];
        for (int i = 0; i < idx.length; i++) {
            double dx = centroids[idx[i]][0] - meanX;
            double dy = centroids[idx[i]][1] - meanY;
            double dz = centroids[idx[i]][2] - meanZ;
            px2d[i] = dx * u[0] + dy * u[1] + dz * u[2];
            py2d[i] = dx * vx + dy * vy + dz * vz;
        }

        // Fit circle in 2D: algebraic least squares
        double[] circle = fitCircle2D(px2d, py2d);
        if (circle == null) return null;

        // Convert 2D circle center back to 3D
        double centerX = meanX + circle[0] * u[0] + circle[1] * vx;
        double centerY = meanY + circle[0] * u[1] + circle[1] * vy;
        double centerZ = meanZ + circle[0] * u[2] + circle[1] * vz;
        double radius = circle[2];

        if (radius < 1e-12 || !Double.isFinite(radius)) return null;

        // Compute height as range of projections along axis
        double minT = Double.MAX_VALUE, maxT = -Double.MAX_VALUE;
        for (int i : idx) {
            double t = (centroids[i][0] - centerX) * axisX
                    + (centroids[i][1] - centerY) * axisY
                    + (centroids[i][2] - centerZ) * axisZ;
            minT = Math.min(minT, t);
            maxT = Math.max(maxT, t);
        }
        double height = maxT - minT;

        // Adjust center point to bottom of cylinder
        double px = centerX + minT * axisX;
        double py = centerY + minT * axisY;
        double pz = centerZ + minT * axisZ;

        return new double[]{px, py, pz, axisX, axisY, axisZ, radius, height};
    }

    // ────────────────────────── CONE ──────────────────────────

    /**
     * Fit cone by estimating apex from line intersections along face normals.
     * Parameters: [ax, ay, az, dx, dy, dz, halfAngle, height]
     */
    private double[] fitCone(double[][] centroids, double[][] normals, int[] idx) {
        // Estimate apex: least-squares intersection of lines (centroid + t*normal)
        // Each line i: p_i + t * n_i. Minimize sum of squared distances to apex.
        // Solution via normal equations: A^T A x = A^T b
        double[][] ata = new double[3][3];
        double[] atb = new double[3];

        for (int k = 0; k < idx.length; k++) {
            double[] p = centroids[idx[k]];
            double[] n = normals[idx[k]];

            // Distance from apex c to line (p, n): ||(c - p) - ((c-p)·n)n||
            // Minimizing over c: (I - nn^T)(c - p) -> least squares
            double nn0 = n[0], nn1 = n[1], nn2 = n[2];

            // M = I - nn^T
            double m00 = 1 - nn0 * nn0, m01 = -nn0 * nn1, m02 = -nn0 * nn2;
            double m10 = -nn1 * nn0, m11 = 1 - nn1 * nn1, m12 = -nn1 * nn2;
            double m20 = -nn2 * nn0, m21 = -nn2 * nn1, m22 = 1 - nn2 * nn2;

            // A^T A += M^T M = M^2 (since M is symmetric)
            ata[0][0] += m00 * m00 + m10 * m10 + m20 * m20;
            ata[0][1] += m00 * m01 + m10 * m11 + m20 * m21;
            ata[0][2] += m00 * m02 + m10 * m12 + m20 * m22;
            ata[1][0] += m01 * m00 + m11 * m10 + m21 * m20;
            ata[1][1] += m01 * m01 + m11 * m11 + m21 * m21;
            ata[1][2] += m01 * m02 + m11 * m12 + m21 * m22;
            ata[2][0] += m02 * m00 + m12 * m10 + m22 * m20;
            ata[2][1] += m02 * m01 + m12 * m11 + m22 * m21;
            ata[2][2] += m02 * m02 + m12 * m12 + m22 * m22;

            // A^T b += M^T M p = M^2 p
            double mp0 = m00 * p[0] + m01 * p[1] + m02 * p[2];
            double mp1 = m10 * p[0] + m11 * p[1] + m12 * p[2];
            double mp2 = m20 * p[0] + m21 * p[1] + m22 * p[2];

            atb[0] += m00 * mp0 + m10 * mp1 + m20 * mp2;
            atb[1] += m01 * mp0 + m11 * mp1 + m21 * mp2;
            atb[2] += m02 * mp0 + m12 * mp1 + m22 * mp2;
        }

        double[] apex = solve3x3(ata, atb);
        if (apex == null) return null;

        // Axis direction: from apex to centroid of sample points
        double meanX = 0, meanY = 0, meanZ = 0;
        for (int i : idx) {
            meanX += centroids[i][0];
            meanY += centroids[i][1];
            meanZ += centroids[i][2];
        }
        meanX /= idx.length;
        meanY /= idx.length;
        meanZ /= idx.length;

        double dx = meanX - apex[0];
        double dy = meanY - apex[1];
        double dz = meanZ - apex[2];
        double dlen = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dlen < 1e-12) return null;
        dx /= dlen;
        dy /= dlen;
        dz /= dlen;

        // Half-angle: average angle between (point - apex) vectors and axis
        double sumAngle = 0;
        double maxT = 0;
        for (int i : idx) {
            double vx = centroids[i][0] - apex[0];
            double vy = centroids[i][1] - apex[1];
            double vz = centroids[i][2] - apex[2];
            double vlen = Math.sqrt(vx * vx + vy * vy + vz * vz);
            if (vlen < 1e-12) continue;
            double cosAngle = (vx * dx + vy * dy + vz * dz) / vlen;
            cosAngle = Math.max(-1.0, Math.min(1.0, cosAngle));
            sumAngle += Math.acos(cosAngle);

            // Project along axis for height
            double t = vx * dx + vy * dy + vz * dz;
            maxT = Math.max(maxT, t);
        }

        double halfAngle = sumAngle / idx.length;
        if (halfAngle < 1e-6 || halfAngle > Math.PI / 2 - 1e-6) return null;

        return new double[]{apex[0], apex[1], apex[2], dx, dy, dz, halfAngle, maxT};
    }

    // ────────────────────────────────────────────────────────────────
    // Complexity penalty (Occam's razor for model selection)
    // ────────────────────────────────────────────────────────────────

    /**
     * Returns a complexity penalty proportional to region size.
     * Simpler models (PLANE) get no penalty; more complex ones pay a tax
     * that must be overcome by genuinely higher inlier counts.
     */
    private double complexityPenaltyFor(PrimitiveType type, int regionSize) {
        // Penalty as fraction of region size
        return switch (type) {
            case PLANE -> 0.0;
            case SPHERE -> 0.10 * regionSize;   // needs 10% more inliers than plane
            case CYLINDER -> 0.12 * regionSize;  // needs 12% more
            case CONE -> 0.15 * regionSize;      // needs 15% more
            case FREEFORM -> 0.0;
        };
    }

    // ────────────────────────────────────────────────────────────────
    // Normal consistency validation
    // ────────────────────────────────────────────────────────────────

    /**
     * Checks how well face normals agree with the fitted primitive geometry.
     * Returns fraction [0..1] of inlier faces whose normals are consistent.
     *
     * - PLANE: normal should be parallel to plane normal (|dot| > threshold)
     * - SPHERE: normal should point radially from center (|dot with radial| > threshold)
     * - CYLINDER: normal should be perpendicular to axis (|dot with axis| < threshold)
     * - CONE: normal should be perpendicular to cone surface
     */
    private double computeNormalConsistency(PrimitiveType type, double[] params,
                                             double[][] centroids, double[][] normals,
                                             int n, double distanceThreshold) {
        if (type == PrimitiveType.PLANE || type == PrimitiveType.FREEFORM) {
            return checkPlaneNormals(params, centroids, normals, n, distanceThreshold);
        } else if (type == PrimitiveType.SPHERE) {
            return checkSphereNormals(params, centroids, normals, n, distanceThreshold);
        } else if (type == PrimitiveType.CYLINDER) {
            return checkCylinderNormals(params, centroids, normals, n, distanceThreshold);
        } else if (type == PrimitiveType.CONE) {
            return checkConeNormals(params, centroids, normals, n, distanceThreshold);
        }
        return 1.0;
    }

    private double checkPlaneNormals(double[] params, double[][] centroids, double[][] normals,
                                      int n, double distanceThreshold) {
        int inliers = 0, consistent = 0;
        double pnx = params[0], pny = params[1], pnz = params[2];
        // Accumulate normal for variance check
        double sumNx = 0, sumNy = 0, sumNz = 0;
        double sumNxSq = 0, sumNySq = 0, sumNzSq = 0;
        for (int i = 0; i < n; i++) {
            if (distToPlane(params, centroids[i]) <= distanceThreshold) {
                inliers++;
                double dot = Math.abs(normals[i][0] * pnx + normals[i][1] * pny + normals[i][2] * pnz);
                if (dot > 0.9) consistent++;  // within ~26 degrees — tight for a plane
                sumNx += normals[i][0]; sumNy += normals[i][1]; sumNz += normals[i][2];
                sumNxSq += normals[i][0] * normals[i][0];
                sumNySq += normals[i][1] * normals[i][1];
                sumNzSq += normals[i][2] * normals[i][2];
            }
        }
        if (inliers < 3) return 0.0;

        // Check normal variance: on a true plane, normals are nearly identical.
        // Variance = E[n^2] - E[n]^2 for each component, then sum.
        double varX = sumNxSq / inliers - (sumNx / inliers) * (sumNx / inliers);
        double varY = sumNySq / inliers - (sumNy / inliers) * (sumNy / inliers);
        double varZ = sumNzSq / inliers - (sumNz / inliers) * (sumNz / inliers);
        double totalVar = varX + varY + varZ;

        // For a true plane, total variance is ~0 (just tessellation noise).
        // For a curved surface, even a 15° cylinder arc has variance ~0.006.
        if (totalVar > 0.001) {
            return 0.0;  // normals vary — this is a curved surface, not a plane
        }

        return (double) consistent / inliers;
    }

    private double checkSphereNormals(double[] params, double[][] centroids, double[][] normals,
                                       int n, double distanceThreshold) {
        int inliers = 0, consistent = 0;
        double cx = params[0], cy = params[1], cz = params[2];

        // Collect inlier normals for covariance analysis
        double sumNx = 0, sumNy = 0, sumNz = 0;
        List<double[]> inlierNormals = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            if (distToSphere(params, centroids[i]) <= distanceThreshold) {
                inliers++;
                sumNx += normals[i][0]; sumNy += normals[i][1]; sumNz += normals[i][2];
                inlierNormals.add(normals[i]);
                // Radial direction from center to point
                double rx = centroids[i][0] - cx;
                double ry = centroids[i][1] - cy;
                double rz = centroids[i][2] - cz;
                double rlen = Math.sqrt(rx * rx + ry * ry + rz * rz);
                if (rlen > 1e-12) {
                    rx /= rlen; ry /= rlen; rz /= rlen;
                    double dot = Math.abs(normals[i][0] * rx + normals[i][1] * ry + normals[i][2] * rz);
                    if (dot > 0.5) consistent++;
                }
            }
        }
        if (inliers == 0) return 0.0;

        // Check 1: if average normal magnitude is close to 1,
        // normals are all pointing the same way (= plane, not sphere)
        double avgLen = Math.sqrt(sumNx * sumNx + sumNy * sumNy + sumNz * sumNz) / inliers;
        if (avgLen > 0.95) return 0.0;

        // Check 2: normals must spread in 3D, not just 2D.
        // Cylinder normals are coplanar (all perpendicular to the axis).
        // Compute the 3x3 covariance matrix of normals and check its determinant.
        // If det ≈ 0, normals lie in a plane → this is a cylinder, not a sphere.
        if (inliers >= 4) {
            double mnx = sumNx / inliers, mny = sumNy / inliers, mnz = sumNz / inliers;
            double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (double[] ni : inlierNormals) {
                double dx = ni[0] - mnx, dy = ni[1] - mny, dz = ni[2] - mnz;
                c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
                c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
            }
            c00 /= inliers; c01 /= inliers; c02 /= inliers;
            c11 /= inliers; c12 /= inliers; c22 /= inliers;

            // Determinant of the covariance matrix = product of eigenvalues
            // If det ≈ 0, the normals are coplanar (one eigenvalue ≈ 0)
            double det = c00 * (c11 * c22 - c12 * c12)
                       - c01 * (c01 * c22 - c12 * c02)
                       + c02 * (c01 * c12 - c11 * c02);

            // Also compute trace (sum of eigenvalues) for normalization
            double trace = c00 + c11 + c22;
            if (trace > 1e-12) {
                // Normalized determinant: det / trace^3 measures "3D-ness"
                // For 3D spread: ~0.037 (equal eigenvalues)
                // For 2D spread: ~0 (one eigenvalue near 0)
                double normalizedDet = det / (trace * trace * trace);
                if (normalizedDet < 0.005) {
                    log.debug("Sphere rejected: normals are coplanar (normalizedDet={}, likely a cylinder)",
                            String.format("%.6f", normalizedDet));
                    return 0.0;
                }
            }
        }

        return (double) consistent / inliers;
    }

    private double checkCylinderNormals(double[] params, double[][] centroids, double[][] normals,
                                         int n, double distanceThreshold) {
        int inliers = 0, consistent = 0;
        double ax = params[3], ay = params[4], az = params[5];
        double px = params[0], py = params[1], pz = params[2];
        for (int i = 0; i < n; i++) {
            if (distToCylinder(params, centroids[i]) <= distanceThreshold) {
                inliers++;
                // Normal should be perpendicular to cylinder axis
                double dotAxis = Math.abs(normals[i][0] * ax + normals[i][1] * ay + normals[i][2] * az);
                if (dotAxis < 0.5) {
                    // Additionally check that normal points roughly radially
                    double dx = centroids[i][0] - px, dy = centroids[i][1] - py, dz = centroids[i][2] - pz;
                    double proj = dx * ax + dy * ay + dz * az;
                    double rx = dx - proj * ax, ry = dy - proj * ay, rz = dz - proj * az;
                    double rlen = Math.sqrt(rx * rx + ry * ry + rz * rz);
                    if (rlen > 1e-12) {
                        rx /= rlen; ry /= rlen; rz /= rlen;
                        double dotRadial = Math.abs(normals[i][0] * rx + normals[i][1] * ry + normals[i][2] * rz);
                        if (dotRadial > 0.4) consistent++;
                    }
                }
            }
        }
        return inliers > 0 ? (double) consistent / inliers : 0.0;
    }

    private double checkConeNormals(double[] params, double[][] centroids, double[][] normals,
                                     int n, double distanceThreshold) {
        int inliers = 0, consistent = 0;
        double apexX = params[0], apexY = params[1], apexZ = params[2];
        double ax = params[3], ay = params[4], az = params[5];
        double halfAngle = params[6];

        for (int i = 0; i < n; i++) {
            if (distToCone(params, centroids[i]) <= distanceThreshold) {
                inliers++;
                // Vector from apex to point
                double vx = centroids[i][0] - apexX;
                double vy = centroids[i][1] - apexY;
                double vz = centroids[i][2] - apexZ;
                double vlen = Math.sqrt(vx * vx + vy * vy + vz * vz);
                if (vlen < 1e-12) continue;

                // Expected normal: perpendicular to cone surface
                // The normal dot axis should be approximately sin(halfAngle)
                double dotAxis = Math.abs(normals[i][0] * ax + normals[i][1] * ay + normals[i][2] * az);
                double expectedDotAxis = Math.sin(halfAngle);
                if (Math.abs(dotAxis - expectedDotAxis) < 0.4) consistent++;
            }
        }
        return inliers > 0 ? (double) consistent / inliers : 0.0;
    }

    // ────────────────────────────────────────────────────────────────
    // Cylinder angular coverage
    // ────────────────────────────────────────────────────────────────

    /**
     * Compute how many degrees of the full 360° circle the inlier faces span
     * around the cylinder axis. Projects inlier centroids onto the plane
     * perpendicular to the axis and measures the angular range.
     *
     * @return angular coverage in degrees [0..360]
     */
    private double computeCylinderAngularCoverage(double[] params, double[][] centroids,
                                                    int n, double distanceThreshold) {
        double px = params[0], py = params[1], pz = params[2];
        double ax = params[3], ay = params[4], az = params[5];

        // Build local 2D coordinate system perpendicular to axis
        double[] u = perpendicular(ax, ay, az);
        double vx = ay * u[2] - az * u[1];
        double vy = az * u[0] - ax * u[2];
        double vz = ax * u[1] - ay * u[0];

        // Collect angles of inlier points around the axis
        List<Double> angles = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (distToCylinder(params, centroids[i]) <= distanceThreshold) {
                // Vector from axis point to centroid, projected onto perpendicular plane
                double dx = centroids[i][0] - px;
                double dy = centroids[i][1] - py;
                double dz = centroids[i][2] - pz;
                double projU = dx * u[0] + dy * u[1] + dz * u[2];
                double projV = dx * vx + dy * vy + dz * vz;
                double angle = Math.toDegrees(Math.atan2(projV, projU));
                if (angle < 0) angle += 360.0;
                angles.add(angle);
            }
        }

        if (angles.size() < 3) return 0.0;

        // Sort angles and find the largest gap
        angles.sort(Double::compareTo);
        double maxGap = 0;
        for (int i = 1; i < angles.size(); i++) {
            maxGap = Math.max(maxGap, angles.get(i) - angles.get(i - 1));
        }
        // Wrap-around gap
        maxGap = Math.max(maxGap, 360.0 - angles.get(angles.size() - 1) + angles.get(0));

        // Coverage = 360 - largest gap
        return 360.0 - maxGap;
    }

    // ────────────────────────────────────────────────────────────────
    // Geometric plausibility checks
    // ────────────────────────────────────────────────────────────────

    /**
     * Reject degenerate primitive fits based on geometric constraints.
     */
    private boolean isGeometricallyPlausible(PrimitiveType type, double[] params, double bboxDiag) {
        if (bboxDiag < 1e-12) return false;

        return switch (type) {
            case PLANE -> true;  // planes are always plausible
            case SPHERE -> {
                double r = params[3];
                // Reject if radius is tiny or absurdly large relative to region
                yield r > bboxDiag * 0.01 && r < bboxDiag * 50.0 && Double.isFinite(r);
            }
            case CYLINDER -> {
                double r = params[6];
                double h = params[7];
                // Reject tiny/huge radius, zero/negative height
                yield r > bboxDiag * 0.005 && r < bboxDiag * 20.0
                        && h > bboxDiag * 0.01 && Double.isFinite(r) && Double.isFinite(h);
            }
            case CONE -> {
                double halfAngle = params[6];
                double h = params[7];
                // Reject extreme half-angles (too flat or too pointy) and degenerate heights
                yield halfAngle > Math.toRadians(5.0) && halfAngle < Math.toRadians(60.0)
                        && h > bboxDiag * 0.01 && Double.isFinite(h);
            }
            case FREEFORM -> true;
        };
    }

    // ────────────────────────────────────────────────────────────────
    // Distance to primitive
    // ────────────────────────────────────────────────────────────────

    private double distanceTo(PrimitiveType type, double[] params, double[] point) {
        return switch (type) {
            case PLANE -> distToPlane(params, point);
            case SPHERE -> distToSphere(params, point);
            case CYLINDER -> distToCylinder(params, point);
            case CONE -> distToCone(params, point);
            case FREEFORM -> Double.MAX_VALUE;
        };
    }

    private double distToPlane(double[] p, double[] pt) {
        // |dot(normal, point) - distance|
        return Math.abs(p[0] * pt[0] + p[1] * pt[1] + p[2] * pt[2] - p[3]);
    }

    private double distToSphere(double[] p, double[] pt) {
        double dx = pt[0] - p[0], dy = pt[1] - p[1], dz = pt[2] - p[2];
        return Math.abs(Math.sqrt(dx * dx + dy * dy + dz * dz) - p[3]);
    }

    private double distToCylinder(double[] p, double[] pt) {
        // Axis point: (p[0], p[1], p[2]), direction: (p[3], p[4], p[5]), radius: p[6]
        double dx = pt[0] - p[0], dy = pt[1] - p[1], dz = pt[2] - p[2];
        double dot = dx * p[3] + dy * p[4] + dz * p[5];
        double perpX = dx - dot * p[3];
        double perpY = dy - dot * p[4];
        double perpZ = dz - dot * p[5];
        double perpDist = Math.sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);
        return Math.abs(perpDist - p[6]);
    }

    private double distToCone(double[] p, double[] pt) {
        // Apex: (p[0],p[1],p[2]), axis: (p[3],p[4],p[5]), halfAngle: p[6]
        double vx = pt[0] - p[0], vy = pt[1] - p[1], vz = pt[2] - p[2];
        double vlen = Math.sqrt(vx * vx + vy * vy + vz * vz);
        if (vlen < 1e-12) return 0; // at apex

        double cosAngle = (vx * p[3] + vy * p[4] + vz * p[5]) / vlen;
        cosAngle = Math.max(-1.0, Math.min(1.0, cosAngle));
        double angle = Math.acos(cosAngle);
        // Distance proportional to angular deviation from cone surface
        return vlen * Math.abs(Math.sin(angle - p[6]));
    }

    // ────────────────────────────────────────────────────────────────
    // Linear algebra utilities
    // ────────────────────────────────────────────────────────────────

    /**
     * Solve a 3x3 linear system Ax = b using Cramer's rule.
     * Returns null if the system is singular.
     */
    private double[] solve3x3(double[][] A, double[] b) {
        double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

        if (Math.abs(det) < 1e-14) return null;

        double invDet = 1.0 / det;

        double x = (b[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                - A[0][1] * (b[1] * A[2][2] - A[1][2] * b[2])
                + A[0][2] * (b[1] * A[2][1] - A[1][1] * b[2])) * invDet;

        double y = (A[0][0] * (b[1] * A[2][2] - A[1][2] * b[2])
                - b[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                + A[0][2] * (A[1][0] * b[2] - b[1] * A[2][0])) * invDet;

        double z = (A[0][0] * (A[1][1] * b[2] - b[1] * A[2][1])
                - A[0][1] * (A[1][0] * b[2] - b[1] * A[2][0])
                + b[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) * invDet;

        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) return null;
        return new double[]{x, y, z};
    }

    /**
     * Find a unit vector perpendicular to the given direction.
     */
    private double[] perpendicular(double dx, double dy, double dz) {
        // Pick the component with smallest absolute value and cross with that axis
        double ax = Math.abs(dx), ay = Math.abs(dy), az = Math.abs(dz);
        double ux, uy, uz;
        if (ax <= ay && ax <= az) {
            // Cross with X axis
            ux = 0;
            uy = -dz;
            uz = dy;
        } else if (ay <= az) {
            // Cross with Y axis
            ux = dz;
            uy = 0;
            uz = -dx;
        } else {
            // Cross with Z axis
            ux = -dy;
            uy = dx;
            uz = 0;
        }
        double len = Math.sqrt(ux * ux + uy * uy + uz * uz);
        return new double[]{ux / len, uy / len, uz / len};
    }

    /**
     * Fit a circle to 2D points using algebraic least squares.
     * Returns [cx, cy, radius] or null if degenerate.
     */
    private double[] fitCircle2D(double[] x, double[] y) {
        int n = x.length;
        if (n < 3) return null;

        // Solve for [a, b, c] in: x^2 + y^2 + a*x + b*y + c = 0
        // Linear system: [x_i, y_i, 1] * [a, b, c]^T = -[x_i^2 + y_i^2]
        // Use normal equations: A^T A [a,b,c] = A^T b
        double s_x = 0, s_y = 0, s_xx = 0, s_yy = 0, s_xy = 0;
        double s_xr = 0, s_yr = 0, s_r = 0;

        for (int i = 0; i < n; i++) {
            double r = x[i] * x[i] + y[i] * y[i];
            s_x += x[i];
            s_y += y[i];
            s_xx += x[i] * x[i];
            s_yy += y[i] * y[i];
            s_xy += x[i] * y[i];
            s_xr += x[i] * r;
            s_yr += y[i] * r;
            s_r += r;
        }

        double[][] A = {
                {s_xx, s_xy, s_x},
                {s_xy, s_yy, s_y},
                {s_x, s_y, n}
        };
        double[] rhs = {-s_xr, -s_yr, -s_r};

        double[] sol = solve3x3(A, rhs);
        if (sol == null) return null;

        double cx = -sol[0] / 2.0;
        double cy = -sol[1] / 2.0;
        double rSq = cx * cx + cy * cy - sol[2];
        if (rSq < 1e-12) return null;

        return new double[]{cx, cy, Math.sqrt(rSq)};
    }
}
