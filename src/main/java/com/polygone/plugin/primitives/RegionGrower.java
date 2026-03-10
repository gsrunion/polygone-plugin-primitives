package com.polygone.plugin.primitives;

import com.polygone.api.model.FaceRegion;
import com.polygone.api.model.MeshData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;

/**
 * Enhanced region growing algorithm for mesh segmentation.
 *
 * <p>Replaces the simpler CurvatureAnalyzer logic with additional criteria:
 * per-face curvature estimation, curvature-difference gating, and optional
 * planarity checks against each region's best-fit plane.
 *
 * <p>Pure computational class — no UI or Spring dependencies.
 */
public class RegionGrower {

    private static final Logger log = LoggerFactory.getLogger(RegionGrower.class);

    /**
     * Grow face regions on the given mesh using BFS with configurable thresholds.
     *
     * @param mesh               the input mesh
     * @param angleThresholdDeg  maximum dihedral angle between adjacent face normals (degrees)
     * @param curvatureThreshold maximum curvature difference between adjacent faces
     * @param planarityThreshold maximum distance from candidate face centroid to region's
     *                           best-fit plane; use {@link Double#MAX_VALUE} to disable
     * @return list of face regions sorted by region id
     */
    public List<FaceRegion> grow(MeshData mesh, double angleThresholdDeg,
                                  double curvatureThreshold, double planarityThreshold) {
        if (mesh == null || mesh.getFaceCount() == 0) {
            return List.of();
        }

        float[] vertices = mesh.getVertices();
        int[] faces = mesh.getFaces();
        int faceCount = mesh.getFaceCount();

        // Step 1: compute per-face normals
        double[][] faceNormals = computeFaceNormals(vertices, faces, faceCount);

        // Step 2: compute per-face centroids
        double[][] faceCentroids = computeFaceCentroids(vertices, faces, faceCount);

        // Step 3: build face adjacency from shared edges
        Map<Integer, List<Integer>> adjacency = buildAdjacencyMap(faces, faceCount);

        // Step 4: compute per-vertex curvature estimates, then per-face curvature
        double[] faceCurvatures = computeFaceCurvatures(vertices, faces, faceCount, faceNormals);

        // Step 5: BFS region growing
        double angleThresholdRad = Math.toRadians(angleThresholdDeg);
        boolean[] visited = new boolean[faceCount];
        List<FaceRegion> regions = new ArrayList<>();
        int regionId = 0;

        for (int seed = 0; seed < faceCount; seed++) {
            if (visited[seed]) {
                continue;
            }

            List<Integer> regionFaces = new ArrayList<>();
            Queue<Integer> queue = new ArrayDeque<>();
            queue.add(seed);
            visited[seed] = true;

            // Running sums for region normal and centroid (for best-fit plane)
            double sumNx = 0, sumNy = 0, sumNz = 0;
            double sumCx = 0, sumCy = 0, sumCz = 0;
            double sumCurvature = 0;

            while (!queue.isEmpty()) {
                int current = queue.poll();
                regionFaces.add(current);

                double[] cn = faceNormals[current];
                double[] cc = faceCentroids[current];
                sumNx += cn[0];
                sumNy += cn[1];
                sumNz += cn[2];
                sumCx += cc[0];
                sumCy += cc[1];
                sumCz += cc[2];
                sumCurvature += faceCurvatures[current];

                // Region's current best-fit plane: average normal + centroid
                int count = regionFaces.size();
                double avgNx = sumNx / count;
                double avgNy = sumNy / count;
                double avgNz = sumNz / count;
                double nLen = Math.sqrt(avgNx * avgNx + avgNy * avgNy + avgNz * avgNz);
                double planeNx = 0, planeNy = 0, planeNz = 0;
                if (nLen > 1e-12) {
                    planeNx = avgNx / nLen;
                    planeNy = avgNy / nLen;
                    planeNz = avgNz / nLen;
                }
                double planeCx = sumCx / count;
                double planeCy = sumCy / count;
                double planeCz = sumCz / count;

                List<Integer> neighbors = adjacency.getOrDefault(current, List.of());
                for (int neighbor : neighbors) {
                    if (visited[neighbor]) {
                        continue;
                    }

                    double[] nn = faceNormals[neighbor];

                    // Criterion 1: dihedral angle between adjacent face normals
                    double dihedralAngle = angleBetween(faceNormals[current], nn);
                    if (dihedralAngle > angleThresholdRad) {
                        continue;
                    }

                    // Criterion 2: curvature difference
                    double curvDiff = Math.abs(faceCurvatures[current] - faceCurvatures[neighbor]);
                    if (curvDiff > curvatureThreshold) {
                        continue;
                    }

                    // Criterion 3: planarity — distance from candidate centroid to region plane
                    if (planarityThreshold < Double.MAX_VALUE) {
                        double[] nc = faceCentroids[neighbor];
                        double dx = nc[0] - planeCx;
                        double dy = nc[1] - planeCy;
                        double dz = nc[2] - planeCz;
                        double dist = Math.abs(planeNx * dx + planeNy * dy + planeNz * dz);
                        if (dist > planarityThreshold) {
                            continue;
                        }
                    }

                    visited[neighbor] = true;
                    queue.add(neighbor);
                }
            }

            // Build the FaceRegion record
            int count = regionFaces.size();
            int[] faceIndices = regionFaces.stream().mapToInt(Integer::intValue).toArray();

            // Normalized average normal
            double nLen = Math.sqrt(sumNx * sumNx + sumNy * sumNy + sumNz * sumNz);
            double[] avgNormal;
            if (nLen > 1e-12) {
                avgNormal = new double[]{sumNx / nLen, sumNy / nLen, sumNz / nLen};
            } else {
                avgNormal = new double[]{0, 0, 1};
            }

            double avgCurvature = sumCurvature / count;
            double[] centroid = new double[]{sumCx / count, sumCy / count, sumCz / count};

            regions.add(new FaceRegion(regionId, faceIndices, avgNormal, avgCurvature, centroid));
            regionId++;
        }

        log.info("Region growing: {} faces -> {} regions (angle={}°, curvature={}, planarity={})",
                faceCount,
                regions.size(),
                angleThresholdDeg,
                curvatureThreshold,
                planarityThreshold == Double.MAX_VALUE ? "off" : planarityThreshold);
        return regions;
    }

    // ────────────────────────────────────────────────────────────────
    // Per-face normals
    // ────────────────────────────────────────────────────────────────

    private double[][] computeFaceNormals(float[] vertices, int[] faces, int faceCount) {
        double[][] normals = new double[faceCount][3];
        for (int f = 0; f < faceCount; f++) {
            int base = f * 6;
            int vi0 = faces[base] * 3;
            int vi1 = faces[base + 2] * 3;
            int vi2 = faces[base + 4] * 3;

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
                normals[f][0] = nx / len;
                normals[f][1] = ny / len;
                normals[f][2] = nz / len;
            }
        }
        return normals;
    }

    // ────────────────────────────────────────────────────────────────
    // Per-face centroids
    // ────────────────────────────────────────────────────────────────

    private double[][] computeFaceCentroids(float[] vertices, int[] faces, int faceCount) {
        double[][] centroids = new double[faceCount][3];
        for (int f = 0; f < faceCount; f++) {
            int base = f * 6;
            int vi0 = faces[base] * 3;
            int vi1 = faces[base + 2] * 3;
            int vi2 = faces[base + 4] * 3;

            centroids[f][0] = (vertices[vi0] + vertices[vi1] + vertices[vi2]) / 3.0;
            centroids[f][1] = (vertices[vi0 + 1] + vertices[vi1 + 1] + vertices[vi2 + 1]) / 3.0;
            centroids[f][2] = (vertices[vi0 + 2] + vertices[vi1 + 2] + vertices[vi2 + 2]) / 3.0;
        }
        return centroids;
    }

    // ────────────────────────────────────────────────────────────────
    // Face adjacency from shared edges
    // ────────────────────────────────────────────────────────────────

    private Map<Integer, List<Integer>> buildAdjacencyMap(int[] faces, int faceCount) {
        Map<Long, List<Integer>> edgeToFaces = new HashMap<>();

        for (int f = 0; f < faceCount; f++) {
            int base = f * 6;
            int v0 = faces[base];
            int v1 = faces[base + 2];
            int v2 = faces[base + 4];

            addEdgeFace(edgeToFaces, v0, v1, f);
            addEdgeFace(edgeToFaces, v1, v2, f);
            addEdgeFace(edgeToFaces, v2, v0, f);
        }

        Map<Integer, List<Integer>> adjacency = new HashMap<>();
        for (List<Integer> edgeFaces : edgeToFaces.values()) {
            for (int i = 0; i < edgeFaces.size(); i++) {
                for (int j = i + 1; j < edgeFaces.size(); j++) {
                    int fa = edgeFaces.get(i);
                    int fb = edgeFaces.get(j);
                    adjacency.computeIfAbsent(fa, k -> new ArrayList<>()).add(fb);
                    adjacency.computeIfAbsent(fb, k -> new ArrayList<>()).add(fa);
                }
            }
        }
        return adjacency;
    }

    private void addEdgeFace(Map<Long, List<Integer>> edgeToFaces, int va, int vb, int faceIndex) {
        long key = encodeEdge(Math.min(va, vb), Math.max(va, vb));
        edgeToFaces.computeIfAbsent(key, k -> new ArrayList<>(2)).add(faceIndex);
    }

    private long encodeEdge(int a, int b) {
        return ((long) a << 32) | (b & 0xFFFFFFFFL);
    }

    // ────────────────────────────────────────────────────────────────
    // Per-face curvature estimation
    // ────────────────────────────────────────────────────────────────

    /**
     * Approximate per-vertex curvature by measuring the variance of face-normal
     * dot products around each vertex. Per-face curvature is the average of its
     * three vertex curvatures.
     */
    private double[] computeFaceCurvatures(float[] vertices, int[] faces, int faceCount,
                                            double[][] faceNormals) {
        int vertexCount = vertices.length / 3;

        // Gather which faces touch each vertex
        @SuppressWarnings("unchecked")
        List<Integer>[] vertexFaces = new List[vertexCount];
        for (int v = 0; v < vertexCount; v++) {
            vertexFaces[v] = new ArrayList<>();
        }
        for (int f = 0; f < faceCount; f++) {
            int base = f * 6;
            vertexFaces[faces[base]].add(f);
            vertexFaces[faces[base + 2]].add(f);
            vertexFaces[faces[base + 4]].add(f);
        }

        // Compute per-vertex curvature as variance of dot products between face normals
        double[] vertexCurvature = new double[vertexCount];
        for (int v = 0; v < vertexCount; v++) {
            List<Integer> vFaces = vertexFaces[v];
            int n = vFaces.size();
            if (n < 2) {
                vertexCurvature[v] = 0;
                continue;
            }

            // Compute all pairwise dot products and their variance
            double sumDot = 0;
            double sumDotSq = 0;
            int pairs = 0;
            for (int i = 0; i < n; i++) {
                double[] ni = faceNormals[vFaces.get(i)];
                for (int j = i + 1; j < n; j++) {
                    double[] nj = faceNormals[vFaces.get(j)];
                    double dot = ni[0] * nj[0] + ni[1] * nj[1] + ni[2] * nj[2];
                    dot = Math.max(-1.0, Math.min(1.0, dot));
                    sumDot += dot;
                    sumDotSq += dot * dot;
                    pairs++;
                }
            }

            if (pairs > 0) {
                double mean = sumDot / pairs;
                double variance = (sumDotSq / pairs) - (mean * mean);
                // Use 1 - mean as the curvature signal: flat regions have mean ~1, curved ~0 or negative
                // Combine with variance for sensitivity to mixed curvature
                vertexCurvature[v] = (1.0 - mean) + variance;
            }
        }

        // Per-face curvature = average of its 3 vertex curvatures
        double[] faceCurvatures = new double[faceCount];
        for (int f = 0; f < faceCount; f++) {
            int base = f * 6;
            int v0 = faces[base];
            int v1 = faces[base + 2];
            int v2 = faces[base + 4];
            faceCurvatures[f] = (vertexCurvature[v0] + vertexCurvature[v1] + vertexCurvature[v2]) / 3.0;
        }

        return faceCurvatures;
    }

    // ────────────────────────────────────────────────────────────────
    // Angle utilities
    // ────────────────────────────────────────────────────────────────

    private double angleBetween(double[] n1, double[] n2) {
        double dot = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];
        dot = Math.max(-1.0, Math.min(1.0, dot));
        return Math.acos(dot);
    }
}
