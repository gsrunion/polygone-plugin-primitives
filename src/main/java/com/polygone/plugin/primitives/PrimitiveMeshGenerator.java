package com.polygone.plugin.primitives;

import com.polygone.api.model.BoundingBox;
import com.polygone.api.model.MeshData;
import com.polygone.api.model.PrimitiveFit;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Generates clean triangle mesh geometry for fitted geometric primitives.
 * All meshes use JavaFX TriangleMesh face format: 6 ints per face (v0, t0, v1, t1, v2, t2).
 */
public class PrimitiveMeshGenerator {

    private static final Logger log = LoggerFactory.getLogger(PrimitiveMeshGenerator.class);

    private PrimitiveMeshGenerator() {
        // utility class
    }

    // ---------------------------------------------------------------
    // PLANE
    // ---------------------------------------------------------------

    /**
     * Generate a rectangular plane mesh bounded to the given selection bounds.
     *
     * @param normal          plane normal [3]
     * @param distance        signed distance from origin
     * @param selectionBounds bounding box of the selected faces
     * @param subdivisions    grid subdivisions per side (1 = single quad = 2 triangles)
     */
    public static MeshData generatePlane(double[] normal, double distance,
                                          BoundingBox selectionBounds, int subdivisions) {
        if (subdivisions < 1) subdivisions = 1;

        // Normalize the normal
        double nx = normal[0], ny = normal[1], nz = normal[2];
        double len = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-12) {
            log.warn("Degenerate plane normal; defaulting to Y-up");
            nx = 0; ny = 1; nz = 0; len = 1;
        }
        nx /= len; ny /= len; nz /= len;

        // Build orthonormal basis (tangent u, tangent v)
        double[] u = new double[3];
        double[] v = new double[3];
        buildOrthonormalBasis(nx, ny, nz, u, v);

        // Plane center: project selectionBounds center onto the plane
        double cx = (selectionBounds.minX() + selectionBounds.maxX()) / 2.0;
        double cy = (selectionBounds.minY() + selectionBounds.maxY()) / 2.0;
        double cz = (selectionBounds.minZ() + selectionBounds.maxZ()) / 2.0;
        double dot = cx * nx + cy * ny + cz * nz;
        double projDist = dot - distance;
        double pcx = cx - projDist * nx;
        double pcy = cy - projDist * ny;
        double pcz = cz - projDist * nz;

        // Grid extent
        double extent = Math.max(selectionBounds.width(),
                Math.max(selectionBounds.height(), selectionBounds.depth())) * 1.2;
        double halfExtent = extent / 2.0;

        int gridSize = subdivisions + 1;
        int vertexCount = gridSize * gridSize;
        float[] vertices = new float[vertexCount * 3];

        for (int row = 0; row < gridSize; row++) {
            double tv = -halfExtent + (extent * row) / subdivisions;
            for (int col = 0; col < gridSize; col++) {
                double tu = -halfExtent + (extent * col) / subdivisions;
                int idx = (row * gridSize + col) * 3;
                vertices[idx]     = (float) (pcx + tu * u[0] + tv * v[0]);
                vertices[idx + 1] = (float) (pcy + tu * u[1] + tv * v[1]);
                vertices[idx + 2] = (float) (pcz + tu * u[2] + tv * v[2]);
            }
        }

        // Normals (all identical)
        float[] normals = new float[vertexCount * 3];
        for (int i = 0; i < vertexCount; i++) {
            normals[i * 3]     = (float) nx;
            normals[i * 3 + 1] = (float) ny;
            normals[i * 3 + 2] = (float) nz;
        }

        // Faces: 2 triangles per cell
        int faceCount = subdivisions * subdivisions * 2;
        int[] faces = new int[faceCount * 6];
        int fi = 0;
        for (int row = 0; row < subdivisions; row++) {
            for (int col = 0; col < subdivisions; col++) {
                int tl = row * gridSize + col;
                int tr = tl + 1;
                int bl = tl + gridSize;
                int br = bl + 1;

                // Triangle 1: tl, bl, tr
                faces[fi++] = tl; faces[fi++] = 0;
                faces[fi++] = bl; faces[fi++] = 0;
                faces[fi++] = tr; faces[fi++] = 0;

                // Triangle 2: tr, bl, br
                faces[fi++] = tr; faces[fi++] = 0;
                faces[fi++] = bl; faces[fi++] = 0;
                faces[fi++] = br; faces[fi++] = 0;
            }
        }

        float[] texCoords = {0f, 0f};
        BoundingBox bbox = computeBoundingBox(vertices);

        log.debug("Generated plane mesh: {} vertices, {} faces", vertexCount, faceCount);
        return new MeshData(vertices, faces, normals, texCoords, null, bbox);
    }

    // ---------------------------------------------------------------
    // SPHERE
    // ---------------------------------------------------------------

    /**
     * Generate a UV sphere mesh.
     *
     * @param center      sphere center [3]
     * @param radius      sphere radius
     * @param latSegments latitude segments (rings from pole to pole)
     * @param lonSegments longitude segments
     */
    public static MeshData generateSphere(double[] center, double radius,
                                           int latSegments, int lonSegments) {
        if (latSegments < 3) latSegments = 3;
        if (lonSegments < 3) lonSegments = 3;

        double cx = center[0], cy = center[1], cz = center[2];

        // Vertices: (latSegments + 1) rows x (lonSegments + 1) columns
        // Row 0 = north pole, row latSegments = south pole
        int rows = latSegments + 1;
        int cols = lonSegments + 1;
        int vertexCount = rows * cols;
        float[] vertices = new float[vertexCount * 3];
        float[] normals = new float[vertexCount * 3];

        for (int lat = 0; lat <= latSegments; lat++) {
            double theta = Math.PI * lat / latSegments; // 0 at north pole, PI at south
            double sinT = Math.sin(theta);
            double cosT = Math.cos(theta);

            for (int lon = 0; lon <= lonSegments; lon++) {
                double phi = 2.0 * Math.PI * lon / lonSegments;
                double sinP = Math.sin(phi);
                double cosP = Math.cos(phi);

                double nx = sinT * cosP;
                double ny = cosT;
                double nz = sinT * sinP;

                int idx = (lat * cols + lon) * 3;
                vertices[idx]     = (float) (cx + radius * nx);
                vertices[idx + 1] = (float) (cy + radius * ny);
                vertices[idx + 2] = (float) (cz + radius * nz);
                normals[idx]     = (float) nx;
                normals[idx + 1] = (float) ny;
                normals[idx + 2] = (float) nz;
            }
        }

        // Faces
        int faceCount = latSegments * lonSegments * 2;
        int[] faces = new int[faceCount * 6];
        int fi = 0;

        for (int lat = 0; lat < latSegments; lat++) {
            for (int lon = 0; lon < lonSegments; lon++) {
                int current = lat * cols + lon;
                int next = current + cols;

                if (lat == 0) {
                    // North pole: single triangle
                    faces[fi++] = current;       faces[fi++] = 0;
                    faces[fi++] = next;          faces[fi++] = 0;
                    faces[fi++] = next + 1;      faces[fi++] = 0;
                } else if (lat == latSegments - 1) {
                    // South pole: single triangle
                    faces[fi++] = current;       faces[fi++] = 0;
                    faces[fi++] = next;          faces[fi++] = 0;
                    faces[fi++] = current + 1;   faces[fi++] = 0;
                } else {
                    // Quad -> 2 triangles
                    faces[fi++] = current;       faces[fi++] = 0;
                    faces[fi++] = next;          faces[fi++] = 0;
                    faces[fi++] = current + 1;   faces[fi++] = 0;

                    faces[fi++] = current + 1;   faces[fi++] = 0;
                    faces[fi++] = next;          faces[fi++] = 0;
                    faces[fi++] = next + 1;      faces[fi++] = 0;
                }
            }
        }

        // Trim faces array if pole rows produced fewer triangles
        if (fi < faces.length) {
            int[] trimmed = new int[fi];
            System.arraycopy(faces, 0, trimmed, 0, fi);
            faces = trimmed;
        }

        float[] texCoords = {0f, 0f};
        BoundingBox bbox = computeBoundingBox(vertices);

        int fc = faces.length / 6;
        log.debug("Generated sphere mesh: {} vertices, {} faces", vertexCount, fc);
        return new MeshData(vertices, faces, normals, texCoords, null, bbox);
    }

    // ---------------------------------------------------------------
    // CYLINDER
    // ---------------------------------------------------------------

    /**
     * Generate a cylinder mesh with end caps.
     *
     * @param axisPoint a point on the axis [3]
     * @param axisDir   axis direction (will be normalized) [3]
     * @param radius    cylinder radius
     * @param height    cylinder height
     * @param segments  circumferential segments
     */
    public static MeshData generateCylinder(double[] axisPoint, double[] axisDir,
                                             double radius, double height, int segments) {
        if (segments < 3) segments = 3;

        // Normalize axis
        double dx = axisDir[0], dy = axisDir[1], dz = axisDir[2];
        double len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-12) {
            log.warn("Degenerate cylinder axis; defaulting to Y-up");
            dx = 0; dy = 1; dz = 0; len = 1;
        }
        dx /= len; dy /= len; dz /= len;

        // Build orthonormal basis
        double[] u = new double[3];
        double[] v = new double[3];
        buildOrthonormalBasis(dx, dy, dz, u, v);

        // Centers
        double bcx = axisPoint[0] - dx * height / 2.0;
        double bcy = axisPoint[1] - dy * height / 2.0;
        double bcz = axisPoint[2] - dz * height / 2.0;
        double tcx = axisPoint[0] + dx * height / 2.0;
        double tcy = axisPoint[1] + dy * height / 2.0;
        double tcz = axisPoint[2] + dz * height / 2.0;

        // Vertices layout:
        // [0..segments-1]            = bottom ring
        // [segments..2*segments-1]   = top ring
        // [2*segments]               = bottom center
        // [2*segments+1]             = top center
        int vertexCount = 2 * segments + 2;
        float[] vertices = new float[vertexCount * 3];
        float[] normals = new float[vertexCount * 3];

        for (int i = 0; i < segments; i++) {
            double angle = 2.0 * Math.PI * i / segments;
            double cosA = Math.cos(angle);
            double sinA = Math.sin(angle);

            double offX = radius * (cosA * u[0] + sinA * v[0]);
            double offY = radius * (cosA * u[1] + sinA * v[1]);
            double offZ = radius * (cosA * u[2] + sinA * v[2]);

            // Radial normal for sides
            double rnx = cosA * u[0] + sinA * v[0];
            double rny = cosA * u[1] + sinA * v[1];
            double rnz = cosA * u[2] + sinA * v[2];

            // Bottom ring
            int bi = i * 3;
            vertices[bi]     = (float) (bcx + offX);
            vertices[bi + 1] = (float) (bcy + offY);
            vertices[bi + 2] = (float) (bcz + offZ);
            normals[bi]     = (float) rnx;
            normals[bi + 1] = (float) rny;
            normals[bi + 2] = (float) rnz;

            // Top ring
            int ti = (segments + i) * 3;
            vertices[ti]     = (float) (tcx + offX);
            vertices[ti + 1] = (float) (tcy + offY);
            vertices[ti + 2] = (float) (tcz + offZ);
            normals[ti]     = (float) rnx;
            normals[ti + 1] = (float) rny;
            normals[ti + 2] = (float) rnz;
        }

        // Bottom center
        int bcIdx = 2 * segments;
        vertices[bcIdx * 3]     = (float) bcx;
        vertices[bcIdx * 3 + 1] = (float) bcy;
        vertices[bcIdx * 3 + 2] = (float) bcz;
        normals[bcIdx * 3]     = (float) -dx;
        normals[bcIdx * 3 + 1] = (float) -dy;
        normals[bcIdx * 3 + 2] = (float) -dz;

        // Top center
        int tcIdx = 2 * segments + 1;
        vertices[tcIdx * 3]     = (float) tcx;
        vertices[tcIdx * 3 + 1] = (float) tcy;
        vertices[tcIdx * 3 + 2] = (float) tcz;
        normals[tcIdx * 3]     = (float) dx;
        normals[tcIdx * 3 + 1] = (float) dy;
        normals[tcIdx * 3 + 2] = (float) dz;

        // Faces: sides (2 tris per quad) + bottom cap + top cap
        int sideFaces = segments * 2;
        int capFaces = segments * 2;
        int faceCount = sideFaces + capFaces;
        int[] faces = new int[faceCount * 6];
        int fi = 0;

        // Side faces
        for (int i = 0; i < segments; i++) {
            int b0 = i;
            int b1 = (i + 1) % segments;
            int t0 = segments + i;
            int t1 = segments + (i + 1) % segments;

            // Triangle 1: b0, t0, b1
            faces[fi++] = b0; faces[fi++] = 0;
            faces[fi++] = t0; faces[fi++] = 0;
            faces[fi++] = b1; faces[fi++] = 0;

            // Triangle 2: b1, t0, t1
            faces[fi++] = b1; faces[fi++] = 0;
            faces[fi++] = t0; faces[fi++] = 0;
            faces[fi++] = t1; faces[fi++] = 0;
        }

        // Bottom cap (winding: center, next, current — facing -axis)
        for (int i = 0; i < segments; i++) {
            int b0 = i;
            int b1 = (i + 1) % segments;
            faces[fi++] = bcIdx; faces[fi++] = 0;
            faces[fi++] = b1;    faces[fi++] = 0;
            faces[fi++] = b0;    faces[fi++] = 0;
        }

        // Top cap (winding: center, current, next — facing +axis)
        for (int i = 0; i < segments; i++) {
            int t0 = segments + i;
            int t1 = segments + (i + 1) % segments;
            faces[fi++] = tcIdx; faces[fi++] = 0;
            faces[fi++] = t0;    faces[fi++] = 0;
            faces[fi++] = t1;    faces[fi++] = 0;
        }

        float[] texCoords = {0f, 0f};
        BoundingBox bbox = computeBoundingBox(vertices);

        log.debug("Generated cylinder mesh: {} vertices, {} faces", vertexCount, faceCount);
        return new MeshData(vertices, faces, normals, texCoords, null, bbox);
    }

    // ---------------------------------------------------------------
    // CONE
    // ---------------------------------------------------------------

    /**
     * Generate a cone mesh with base cap.
     *
     * @param apex      cone apex point [3]
     * @param axisDir   axis direction from apex toward base (will be normalized) [3]
     * @param halfAngle half-angle in radians
     * @param height    cone height from apex to base
     * @param segments  circumferential segments
     */
    public static MeshData generateCone(double[] apex, double[] axisDir,
                                         double halfAngle, double height, int segments) {
        if (segments < 3) segments = 3;

        // Normalize axis
        double dx = axisDir[0], dy = axisDir[1], dz = axisDir[2];
        double len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-12) {
            log.warn("Degenerate cone axis; defaulting to Y-up");
            dx = 0; dy = 1; dz = 0; len = 1;
        }
        dx /= len; dy /= len; dz /= len;

        double baseRadius = height * Math.tan(halfAngle);

        // Base center
        double bx = apex[0] + dx * height;
        double by = apex[1] + dy * height;
        double bz = apex[2] + dz * height;

        // Build orthonormal basis
        double[] u = new double[3];
        double[] v = new double[3];
        buildOrthonormalBasis(dx, dy, dz, u, v);

        // Vertices layout:
        // [0]                  = apex
        // [1..segments]        = base ring
        // [segments+1]         = base center
        int vertexCount = segments + 2;
        float[] vertices = new float[vertexCount * 3];
        float[] normals = new float[vertexCount * 3];

        // Apex
        vertices[0] = (float) apex[0];
        vertices[1] = (float) apex[1];
        vertices[2] = (float) apex[2];
        // Apex normal points along axis (approximate)
        normals[0] = (float) dx;
        normals[1] = (float) dy;
        normals[2] = (float) dz;

        // Slant normal factor: for a cone, the outward normal tilts away from axis by (PI/2 - halfAngle)
        double cosHA = Math.cos(halfAngle);
        double sinHA = Math.sin(halfAngle);

        // Base ring
        for (int i = 0; i < segments; i++) {
            double angle = 2.0 * Math.PI * i / segments;
            double cosA = Math.cos(angle);
            double sinA = Math.sin(angle);

            double offX = baseRadius * (cosA * u[0] + sinA * v[0]);
            double offY = baseRadius * (cosA * u[1] + sinA * v[1]);
            double offZ = baseRadius * (cosA * u[2] + sinA * v[2]);

            int idx = (1 + i) * 3;
            vertices[idx]     = (float) (bx + offX);
            vertices[idx + 1] = (float) (by + offY);
            vertices[idx + 2] = (float) (bz + offZ);

            // Slant normal: radial component * cosHA - axis * sinHA  (pointing outward from surface)
            double radX = cosA * u[0] + sinA * v[0];
            double radY = cosA * u[1] + sinA * v[1];
            double radZ = cosA * u[2] + sinA * v[2];
            normals[idx]     = (float) (radX * cosHA - dx * sinHA);
            normals[idx + 1] = (float) (radY * cosHA - dy * sinHA);
            normals[idx + 2] = (float) (radZ * cosHA - dz * sinHA);
        }

        // Base center
        int bcIdx = segments + 1;
        vertices[bcIdx * 3]     = (float) bx;
        vertices[bcIdx * 3 + 1] = (float) by;
        vertices[bcIdx * 3 + 2] = (float) bz;
        normals[bcIdx * 3]     = (float) dx;
        normals[bcIdx * 3 + 1] = (float) dy;
        normals[bcIdx * 3 + 2] = (float) dz;

        // Faces: side triangles + base cap
        int faceCount = segments * 2; // segments side + segments cap
        int[] faces = new int[faceCount * 6];
        int fi = 0;

        // Side faces: apex -> ring[i] -> ring[i+1]
        for (int i = 0; i < segments; i++) {
            int r0 = 1 + i;
            int r1 = 1 + (i + 1) % segments;
            faces[fi++] = 0;  faces[fi++] = 0; // apex
            faces[fi++] = r0; faces[fi++] = 0;
            faces[fi++] = r1; faces[fi++] = 0;
        }

        // Base cap (winding: center, next, current — facing +axis away from apex)
        for (int i = 0; i < segments; i++) {
            int r0 = 1 + i;
            int r1 = 1 + (i + 1) % segments;
            faces[fi++] = bcIdx; faces[fi++] = 0;
            faces[fi++] = r1;    faces[fi++] = 0;
            faces[fi++] = r0;    faces[fi++] = 0;
        }

        float[] texCoords = {0f, 0f};
        BoundingBox bbox = computeBoundingBox(vertices);

        log.debug("Generated cone mesh: {} vertices, {} faces", vertexCount, faceCount);
        return new MeshData(vertices, faces, normals, texCoords, null, bbox);
    }

    // ---------------------------------------------------------------
    // FROM FIT (dispatcher)
    // ---------------------------------------------------------------

    /**
     * Generate mesh for a PrimitiveFit using default resolution.
     * Uses the fit parameters and selectionBounds to auto-configure.
     *
     * @param fit             the primitive fit result
     * @param selectionBounds bounding box of the selected faces
     * @return generated MeshData, or null for FREEFORM
     */
    public static MeshData generateFromFit(PrimitiveFit fit, BoundingBox selectionBounds) {
        double[] p = fit.parameters();

        return switch (fit.type()) {
            case PLANE -> {
                double[] normal = {p[0], p[1], p[2]};
                double distance = p[3];
                yield generatePlane(normal, distance, selectionBounds, 1);
            }
            case SPHERE -> {
                double[] center = {p[0], p[1], p[2]};
                double radius = p[3];
                yield generateSphere(center, radius, 32, 16);
            }
            case CYLINDER -> {
                double[] axisPoint = {p[0], p[1], p[2]};
                double[] axisDir = {p[3], p[4], p[5]};
                double radius = p[6];
                double height = p[7];
                yield generateCylinder(axisPoint, axisDir, radius, height, 32);
            }
            case CONE -> {
                double[] apex = {p[0], p[1], p[2]};
                double[] axisDir = {p[3], p[4], p[5]};
                double halfAngle = p[6];
                double height = p[7];
                yield generateCone(apex, axisDir, halfAngle, height, 32);
            }
            case FREEFORM -> {
                log.debug("FREEFORM primitive type; no mesh generated");
                yield null;
            }
        };
    }

    // ---------------------------------------------------------------
    // Utilities
    // ---------------------------------------------------------------

    /**
     * Build an orthonormal basis from a direction vector.
     * Populates u and v as two tangent vectors perpendicular to (dx, dy, dz).
     */
    private static void buildOrthonormalBasis(double dx, double dy, double dz,
                                               double[] u, double[] v) {
        // Choose a vector not parallel to d for the cross product
        double ax, ay, az;
        if (Math.abs(dx) < 0.9) {
            ax = 1; ay = 0; az = 0;
        } else {
            ax = 0; ay = 1; az = 0;
        }

        // u = normalize(cross(d, a))
        double ux = dy * az - dz * ay;
        double uy = dz * ax - dx * az;
        double uz = dx * ay - dy * ax;
        double uLen = Math.sqrt(ux * ux + uy * uy + uz * uz);
        u[0] = ux / uLen;
        u[1] = uy / uLen;
        u[2] = uz / uLen;

        // v = cross(d, u)
        v[0] = dy * u[2] - dz * u[1];
        v[1] = dz * u[0] - dx * u[2];
        v[2] = dx * u[1] - dy * u[0];
    }

    /**
     * Compute an axis-aligned bounding box from a flat vertex array (x, y, z triples).
     */
    private static BoundingBox computeBoundingBox(float[] vertices) {
        if (vertices.length < 3) {
            return new BoundingBox(0, 0, 0, 0, 0, 0);
        }
        double minX = vertices[0], minY = vertices[1], minZ = vertices[2];
        double maxX = minX, maxY = minY, maxZ = minZ;

        for (int i = 3; i < vertices.length; i += 3) {
            double x = vertices[i], y = vertices[i + 1], z = vertices[i + 2];
            if (x < minX) minX = x; else if (x > maxX) maxX = x;
            if (y < minY) minY = y; else if (y > maxY) maxY = y;
            if (z < minZ) minZ = z; else if (z > maxZ) maxZ = z;
        }
        return new BoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
    }
}
