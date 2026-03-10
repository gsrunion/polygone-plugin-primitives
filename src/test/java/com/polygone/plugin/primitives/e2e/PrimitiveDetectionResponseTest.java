package com.polygone.plugin.primitives.e2e;

import com.polygone.api.model.FaceGroup;
import com.polygone.api.model.PrimitiveFit;
import com.polygone.api.model.PrimitiveType;
import com.polygone.app.service.TestHookState;
import javafx.scene.paint.Color;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Verifies that primitive detection results include primitiveType in group details,
 * and that primitiveCount in status reflects detected primitives.
 * Regression test for STAGE2-002.
 */
class PrimitiveDetectionResponseTest {

    @Test
    void stateTracksPrimitiveTypeMappingAfterDetection() {
        TestHookState state = new TestHookState();

        assertTrue(state.getLastGroupPrimitiveTypes().isEmpty());
        assertTrue(state.getLastPrimitiveFits().isEmpty());

        List<PrimitiveFit> fits = List.of(
                new PrimitiveFit(PrimitiveType.PLANE, new double[]{0, 0, 1, 0}, new int[]{0, 1, 2}, 0.01, 0),
                new PrimitiveFit(PrimitiveType.CYLINDER, new double[]{0, 0, 0, 0, 1, 0, 1.0, 4.0}, new int[]{3, 4, 5}, 0.02, 1)
        );
        Map<Integer, PrimitiveType> groupTypeMap = Map.of(0, PrimitiveType.PLANE, 1, PrimitiveType.CYLINDER);

        state.setLastPrimitiveFits(fits);
        state.setLastGroupPrimitiveTypes(groupTypeMap);

        assertEquals(2, state.getLastPrimitiveFits().size());
        assertEquals(PrimitiveType.PLANE, state.getLastGroupPrimitiveTypes().get(0));
        assertEquals(PrimitiveType.CYLINDER, state.getLastGroupPrimitiveTypes().get(1));
    }

    @Test
    void faceGroupAnalysisClearsPrimitiveState() {
        TestHookState state = new TestHookState();

        state.setLastPrimitiveFits(List.of(
                new PrimitiveFit(PrimitiveType.PLANE, new double[]{0, 0, 1, 0}, new int[]{0}, 0.01, 0)
        ));
        state.setLastGroupPrimitiveTypes(Map.of(0, PrimitiveType.PLANE));

        assertEquals(1, state.getLastPrimitiveFits().size());
        assertFalse(state.getLastGroupPrimitiveTypes().isEmpty());

        state.setLastPrimitiveFits(List.of());
        state.setLastGroupPrimitiveTypes(Map.of());

        assertTrue(state.getLastPrimitiveFits().isEmpty());
        assertTrue(state.getLastGroupPrimitiveTypes().isEmpty());
    }

    @Test
    void groupDetailsIncludePrimitiveTypeWhenAvailable() {
        TestHookState state = new TestHookState();

        List<FaceGroup> groups = List.of(
                new FaceGroup(0, "PLANE #0", new int[]{0, 1, 2}, Color.GRAY),
                new FaceGroup(1, "CYLINDER #1", new int[]{3, 4, 5, 6}, Color.GRAY)
        );
        Map<Integer, PrimitiveType> groupTypeMap = Map.of(
                0, PrimitiveType.PLANE,
                1, PrimitiveType.CYLINDER
        );

        state.setLastFaceGroups(groups);
        state.setLastGroupPrimitiveTypes(groupTypeMap);

        List<Map<String, Object>> groupDetails = new ArrayList<>();
        var typeMap = state.getLastGroupPrimitiveTypes();
        for (FaceGroup g : state.getLastFaceGroups()) {
            Map<String, Object> gm = new HashMap<>();
            gm.put("id", g.groupId());
            gm.put("name", g.name());
            gm.put("faceCount", g.faceIndices().length);
            var pt = typeMap.get(g.groupId());
            if (pt != null) {
                gm.put("primitiveType", pt.name());
            }
            groupDetails.add(gm);
        }

        assertEquals(2, groupDetails.size());
        assertEquals("PLANE", groupDetails.get(0).get("primitiveType"));
        assertEquals(3, groupDetails.get(0).get("faceCount"));
        assertEquals("CYLINDER", groupDetails.get(1).get("primitiveType"));
        assertEquals(4, groupDetails.get(1).get("faceCount"));
    }

    @Test
    void groupDetailsOmitPrimitiveTypeForPlainAnalysis() {
        TestHookState state = new TestHookState();

        List<FaceGroup> groups = List.of(
                new FaceGroup(0, "Group 0", new int[]{0, 1, 2}, Color.GRAY)
        );
        state.setLastFaceGroups(groups);
        state.setLastGroupPrimitiveTypes(Map.of());

        var typeMap = state.getLastGroupPrimitiveTypes();
        Map<String, Object> gm = new HashMap<>();
        FaceGroup g = state.getLastFaceGroups().get(0);
        gm.put("id", g.groupId());
        gm.put("name", g.name());
        gm.put("faceCount", g.faceIndices().length);
        var pt = typeMap.get(g.groupId());
        if (pt != null) {
            gm.put("primitiveType", pt.name());
        }

        assertNull(gm.get("primitiveType"));
    }

    @Test
    void primitiveCountInStatusReflectsDetectedPrimitives() {
        TestHookState state = new TestHookState();

        assertEquals(0, state.getLastPrimitiveFits().size());

        state.setLastPrimitiveFits(List.of(
                new PrimitiveFit(PrimitiveType.PLANE, new double[]{0, 0, 1, 0}, new int[]{0, 1}, 0.01, 0),
                new PrimitiveFit(PrimitiveType.CYLINDER, new double[]{0, 0, 0, 0, 1, 0, 1, 4}, new int[]{2, 3}, 0.02, 1),
                new PrimitiveFit(PrimitiveType.PLANE, new double[]{1, 0, 0, 0}, new int[]{4, 5}, 0.01, 2)
        ));

        assertEquals(3, state.getLastPrimitiveFits().size());
    }
}
