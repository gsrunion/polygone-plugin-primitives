package com.polygone.plugin.primitives.e2e;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PrimitivesEndToEndTest {

    @TempDir
    Path tempDir;

    @Test
    @Timeout(value = 2, unit = TimeUnit.MINUTES)
    void runsGroupDirectedPrimitiveDetection() throws Exception {
        Assumptions.assumeTrue(PolyGoneAppHarness.canRunUiProcess(), "No DISPLAY available for JavaFX E2E test");

        Path projectRoot = findProjectRoot();
        Path home = tempDir.resolve("home-primitives");

        try (PolyGoneAppHarness app = PolyGoneAppHarness.launch(projectRoot, home, PolyGoneAppHarness.findFreePort())) {
            assertSuccessfulOpen(app, projectRoot.resolve("testdata/cad-reference/pipe-flange.stl"));

            Map<String, Object> detect = app.post("/api/detect-group-primitives", Map.of(
                    "groupAngle", "35",
                    "minGroupSize", "10",
                    "ransacIters", "400",
                    "distThreshold", "1.0",
                    "minInlierRatio", "0.4",
                    "globfit", "true"
            ));

            assertEquals(Boolean.TRUE, detect.get("success"));
            assertTrue(((Number) detect.get("primitiveCount")).intValue() > 0);

            @SuppressWarnings("unchecked")
            Map<String, Number> typeCounts = (Map<String, Number>) detect.get("typeCounts");
            assertFalse(typeCounts.isEmpty());

            Map<String, Object> groups = app.get("/api/groups", Map.of());
            assertTrue(((List<?>) groups.get("primitives")).size() > 0);
        }
    }

    private void assertSuccessfulOpen(PolyGoneAppHarness app, Path meshFile) throws Exception {
        Map<String, Object> response = app.post("/api/open", Map.of("path", meshFile.toString()));
        assertEquals(Boolean.TRUE, response.get("success"), () -> "Open failed for " + meshFile + ": " + response);
        assertEquals("mesh", response.get("loadedObjectType"));
        assertTrue(((Number) response.get("vertexCount")).intValue() > 0);

        Map<String, Object> status = app.get("/api/status", Map.of());
        assertEquals(Boolean.TRUE, status.get("meshLoaded"));
    }

    private Path findProjectRoot() {
        Path current = Path.of("").toAbsolutePath().normalize();
        while (current != null && !Files.exists(current.resolve("settings.gradle"))) {
            current = current.getParent();
        }
        if (current == null) {
            throw new IllegalStateException("Could not locate project root from " + Path.of("").toAbsolutePath());
        }
        return current;
    }
}
