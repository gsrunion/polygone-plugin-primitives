package com.polygone.plugin.primitives;

import com.polygone.api.model.PrimitiveFit;
import com.polygone.api.model.PrimitiveType;
import com.polygone.api.model.RelationshipType;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

class GlobFitRefinerTest {
    @Test
    void snapsNearlyParallelPlanesAndEqualRadiusCylinders() {
        GlobFitRefiner refiner = new GlobFitRefiner();
        PrimitiveFit p1 = new PrimitiveFit(PrimitiveType.PLANE, new double[]{0, 0, 1, 0}, new int[]{0}, 0, 0);
        PrimitiveFit p2 = new PrimitiveFit(PrimitiveType.PLANE, new double[]{0.03, 0, 0.9995, 1}, new int[]{1}, 0, 1);
        PrimitiveFit c1 = new PrimitiveFit(PrimitiveType.CYLINDER, new double[]{0,0,0, 0,1,0, 1.0, 4}, new int[]{2}, 0, 2);
        PrimitiveFit c2 = new PrimitiveFit(PrimitiveType.CYLINDER, new double[]{0.1,0,0, 0,1,0.02, 1.02, 4}, new int[]{3}, 0, 3);
        var result = refiner.refine(List.of(p1, p2, c1, c2), 5.0, 1.0, 0.05);
        assertTrue(result.relationships().stream().anyMatch(r -> r.type() == RelationshipType.PARALLEL));
        assertTrue(result.relationships().stream().anyMatch(r -> r.type() == RelationshipType.EQUAL_RADIUS));
    }
}
