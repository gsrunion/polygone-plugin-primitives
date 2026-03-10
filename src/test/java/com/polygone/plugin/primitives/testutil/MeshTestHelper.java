package com.polygone.plugin.primitives.testutil;

import com.polygone.api.model.BoundingBox;
import com.polygone.api.model.MeshData;
import com.polygone.plugin.primitives.PrimitiveMeshGenerator;

import java.util.Random;

public final class MeshTestHelper {
    private MeshTestHelper() {}
    public static MeshData createUnitCube() {
        float[] vertices = {
                0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0,
                0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1
        };
        int[] faces = {
                0,0, 1,0, 2,0, 0,0, 2,0, 3,0,
                4,0, 6,0, 5,0, 4,0, 7,0, 6,0,
                0,0, 4,0, 5,0, 0,0, 5,0, 1,0,
                1,0, 5,0, 6,0, 1,0, 6,0, 2,0,
                2,0, 6,0, 7,0, 2,0, 7,0, 3,0,
                3,0, 7,0, 4,0, 3,0, 4,0, 0,0
        };
        return new MeshData(vertices, faces, computeFaceNormals(vertices, faces), new float[]{0, 0}, null,
                new BoundingBox(0, 0, 0, 1, 1, 1));
    }
    public static MeshData createQuad() {
        float[] vertices = {0,0,0, 1,0,0, 1,1,0, 0,1,0};
        int[] faces = {0,0, 1,0, 2,0, 0,0, 2,0, 3,0};
        return new MeshData(vertices, faces, computeFaceNormals(vertices, faces), new float[]{0, 0}, null,
                new BoundingBox(0, 0, 0, 1, 1, 0));
    }
    public static MeshData createSphere(double radius, int segments) { return PrimitiveMeshGenerator.generateSphere(new double[]{0,0,0}, radius, segments, segments * 2); }
    public static MeshData createCylinder(double radius, double height, int segments) { return PrimitiveMeshGenerator.generateCylinder(new double[]{0,0,0}, new double[]{0,1,0}, radius, height, segments); }
    public static MeshData createPlaneGrid(int rows, int cols, double spacing) {
        int vertexCount = (rows + 1) * (cols + 1);
        float[] vertices = new float[vertexCount * 3];
        int[] faces = new int[rows * cols * 12];
        int vi = 0;
        for (int r = 0; r <= rows; r++) {
            for (int c = 0; c <= cols; c++) {
                vertices[vi++] = (float) (c * spacing);
                vertices[vi++] = (float) (r * spacing);
                vertices[vi++] = 0f;
            }
        }
        int fi = 0;
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int tl = r * (cols + 1) + c;
                int tr = tl + 1;
                int bl = tl + cols + 1;
                int br = bl + 1;
                faces[fi++] = tl; faces[fi++] = 0; faces[fi++] = bl; faces[fi++] = 0; faces[fi++] = tr; faces[fi++] = 0;
                faces[fi++] = tr; faces[fi++] = 0; faces[fi++] = bl; faces[fi++] = 0; faces[fi++] = br; faces[fi++] = 0;
            }
        }
        return new MeshData(vertices, faces, computeFaceNormals(vertices, faces), new float[]{0, 0}, null,
                new BoundingBox(0, 0, 0, cols * spacing, rows * spacing, 0));
    }
    public static MeshData addNoise(MeshData mesh, double sigma) {
        float[] vertices = mesh.getVertices().clone();
        Random random = new Random(42);
        for (int i = 0; i < vertices.length; i++) vertices[i] += (float) (random.nextGaussian() * sigma);
        return new MeshData(vertices, mesh.getFaces(), mesh.getNormals(), mesh.getTexCoords(), mesh.getSmoothingGroups(), boundingBox(vertices));
    }
    private static BoundingBox boundingBox(float[] vertices) {
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, minZ = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE, maxZ = -Double.MAX_VALUE;
        for (int i = 0; i < vertices.length; i += 3) {
            minX = Math.min(minX, vertices[i]);
            minY = Math.min(minY, vertices[i + 1]);
            minZ = Math.min(minZ, vertices[i + 2]);
            maxX = Math.max(maxX, vertices[i]);
            maxY = Math.max(maxY, vertices[i + 1]);
            maxZ = Math.max(maxZ, vertices[i + 2]);
        }
        return new BoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
    }
    private static float[] computeFaceNormals(float[] vertices, int[] faces) {
        float[] normals = new float[(faces.length / 6) * 3];
        for (int i = 0; i < faces.length / 6; i++) {
            int base = i * 6;
            int v0 = faces[base] * 3;
            int v1 = faces[base + 2] * 3;
            int v2 = faces[base + 4] * 3;
            float ax = vertices[v1] - vertices[v0];
            float ay = vertices[v1 + 1] - vertices[v0 + 1];
            float az = vertices[v1 + 2] - vertices[v0 + 2];
            float bx = vertices[v2] - vertices[v0];
            float by = vertices[v2 + 1] - vertices[v0 + 1];
            float bz = vertices[v2 + 2] - vertices[v0 + 2];
            float nx = ay * bz - az * by;
            float ny = az * bx - ax * bz;
            float nz = ax * by - ay * bx;
            float len = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
            normals[i * 3] = nx / len;
            normals[i * 3 + 1] = ny / len;
            normals[i * 3 + 2] = nz / len;
        }
        return normals;
    }
}
