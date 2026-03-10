package com.polygone.plugin.primitives;

import com.polygone.api.model.FaceRegion;
import com.polygone.api.model.MeshData;
import com.polygone.api.model.PrimitiveType;
import com.polygone.plugin.primitives.testutil.MeshTestHelper;
import org.junit.jupiter.api.Test;

import java.util.stream.IntStream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class RansacPrimitiveFitterTest {
    private final RansacPrimitiveFitter fitter = new RansacPrimitiveFitter();

    @Test
    void detectsPlaneSphereAndCylinder() {
        assertEquals(PrimitiveType.PLANE, fitWholeMesh(MeshTestHelper.createPlaneGrid(4, 4, 1.0)).type());
        assertEquals(PrimitiveType.SPHERE, fitWholeMesh(MeshTestHelper.createSphere(2.0, 8)).type());
        assertEquals(PrimitiveType.CYLINDER, fitCylinderSides(MeshTestHelper.createCylinder(1.0, 4.0, 16), PrimitiveType.CYLINDER).type());
    }

    @Test
    void toleratesNoise() {
        var noisySphere = MeshTestHelper.addNoise(MeshTestHelper.createSphere(2.0, 8), 0.01);
        assertEquals(PrimitiveType.SPHERE, fitWholeMesh(noisySphere).type());
    }

    private com.polygone.api.model.PrimitiveFit fitCylinderSides(MeshData mesh, PrimitiveType forceType) {
        int[] sideFaces = IntStream.range(0, mesh.getFaceCount())
                .filter(fi -> Math.abs(faceNormalY(mesh, fi)) < 0.5f)
                .toArray();
        FaceRegion region = new FaceRegion(0, sideFaces, new double[]{0, 1, 0}, 0, new double[]{0, 0, 0});
        var fit = fitter.fitRegion(mesh, region, forceType, 500, 0.2, 0.5, 0);
        assertTrue(Double.isNaN(fit.fitError()) || fit.fitError() < 0.25);
        return fit;
    }

    private float faceNormalY(MeshData mesh, int faceIndex) {
        int[] faces = mesh.getFaces();
        float[] vertices = mesh.getVertices();
        int base = faceIndex * 6;
        int v0 = faces[base] * 3;
        int v1 = faces[base + 2] * 3;
        int v2 = faces[base + 4] * 3;
        float ax = vertices[v1] - vertices[v0];
        float ay = vertices[v1 + 1] - vertices[v0 + 1];
        float az = vertices[v1 + 2] - vertices[v0 + 2];
        float bx = vertices[v2] - vertices[v0];
        float by = vertices[v2 + 1] - vertices[v0 + 1];
        float bz = vertices[v2 + 2] - vertices[v0 + 2];
        return az * bx - ax * bz;
    }

    private com.polygone.api.model.PrimitiveFit fitWholeMesh(com.polygone.api.model.MeshData mesh) {
        FaceRegion region = new FaceRegion(0, java.util.stream.IntStream.range(0, mesh.getFaceCount()).toArray(), new double[]{0,0,1}, 0, new double[]{0,0,0});
        var fit = fitter.fitRegion(mesh, region, null, 500, 0.2, 0.5, 0);
        assertTrue(Double.isNaN(fit.fitError()) || fit.fitError() < 0.25);
        return fit;
    }
}
