package com.polygone.plugin.primitives;

import com.polygone.plugin.primitives.testutil.MeshTestHelper;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class RegionGrowerTest {
    @Test
    void thresholdSplitsOrMergesFlatRegions() {
        RegionGrower grower = new RegionGrower();
        var cube = MeshTestHelper.createUnitCube();
        assertEquals(6, grower.grow(cube, 45, 1, Double.MAX_VALUE).size());
        assertEquals(1, grower.grow(cube, 91, 1, Double.MAX_VALUE).size());
    }
}
