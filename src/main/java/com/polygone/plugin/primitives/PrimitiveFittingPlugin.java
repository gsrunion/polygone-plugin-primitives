package com.polygone.plugin.primitives;

import com.polygone.api.MenuContribution;
import com.polygone.api.MeshOperation;
import com.polygone.api.PluginManifest;
import com.polygone.api.PolyGoneContext;
import com.polygone.api.PolyGonePlugin;
import com.polygone.api.PrimitiveFittingProvider;
import com.polygone.api.model.FaceGroup;
import com.polygone.api.model.FaceRegion;
import com.polygone.api.model.GeometricRelationship;
import com.polygone.api.model.MeshData;
import com.polygone.api.model.PrimitiveFit;
import com.polygone.api.model.PrimitiveType;
import javafx.application.Platform;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.stage.Modality;
import javafx.stage.Stage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Built-in plugin that provides the 3-layer primitive detection pipeline:
 * region growing, RANSAC primitive fitting, and GlobFit refinement.
 */
public class PrimitiveFittingPlugin implements PolyGonePlugin, PrimitiveFittingProvider {

    private static final Logger log = LoggerFactory.getLogger(PrimitiveFittingPlugin.class);

    private PolyGoneContext context;
    private final RegionGrower regionGrower = new RegionGrower();
    private final RansacPrimitiveFitter ransacFitter = new RansacPrimitiveFitter();
    private final GlobFitRefiner globFitRefiner = new GlobFitRefiner();

    private MeshData originalMesh;

    @Override
    public PluginManifest getManifest() {
        return new PluginManifest(
                "primitives",
                "Primitive Detection",
                "0.1.0",
                "RANSAC-based primitive detection with GlobFit refinement",
                "PolyGone"
        );
    }

    @Override
    public void initialize(PolyGoneContext context) {
        this.context = context;
        log.info("Primitive Detection plugin initialized");
    }

    @Override
    public void shutdown() {
        log.info("Primitive Detection plugin shutting down");
        this.context = null;
    }

    @Override
    public List<MenuContribution> getMenuContributions() {
        return List.of(
                new MenuContribution("Tools/Primitive Detection", "Detect Primitives...", this::runPrimitiveDetection)
        );
    }

    @Override
    public List<MeshOperation> getMeshOperations() {
        return List.of(new PrimitiveDetectionOperation());
    }

    // ========================= PrimitiveFittingProvider =========================

    @Override
    public List<FaceRegion> growRegions(MeshData mesh, double angleThreshold, double curvatureThreshold, double planarityThreshold) {
        return regionGrower.grow(mesh, angleThreshold, curvatureThreshold, planarityThreshold);
    }

    @Override
    public List<PrimitiveFit> fitPrimitives(MeshData mesh, List<FaceRegion> regions, int ransacIterations, double distanceThreshold, double minInlierRatio, double maxFitError) {
        return ransacFitter.fitAll(mesh, regions, ransacIterations, distanceThreshold, minInlierRatio, maxFitError);
    }

    @Override
    public RefinementResult refinePrimitives(List<PrimitiveFit> fits, double angleToleranceDeg, double distanceTolerance, double radiusToleranceRatio) {
        GlobFitRefiner.RefinementResult refined = globFitRefiner.refine(fits, angleToleranceDeg, distanceTolerance, radiusToleranceRatio);
        return new RefinementResult(refined.primitives(), refined.relationships());
    }

    // ========================= Primitive Detection Dialog =========================

    private void runPrimitiveDetection() {
        if (context == null) {
            log.warn("Plugin not initialized; cannot run primitive detection");
            return;
        }
        MeshData mesh = context.getCurrentMesh();
        if (mesh == null) {
            log.warn("No mesh loaded; cannot run primitive detection");
            return;
        }
        if (originalMesh == null || mesh.getTextureAtlas() == null) {
            originalMesh = mesh;
        }
        showPrimitiveDetectionDialog();
    }

    private void showPrimitiveDetectionDialog() {
        Stage dialog = new Stage();
        dialog.initModality(Modality.NONE);
        dialog.setTitle("Primitive Detection");
        dialog.setAlwaysOnTop(true);

        Slider angleSlider = new Slider(1, 90, 15);
        angleSlider.setShowTickMarks(true);
        angleSlider.setShowTickLabels(true);
        angleSlider.setMajorTickUnit(15);
        angleSlider.setPrefWidth(250);

        TextField angleField = new TextField("15.0");
        angleField.setPrefWidth(50);
        angleSlider.valueProperty().addListener((obs, o, n) -> angleField.setText(String.format("%.1f", n.doubleValue())));

        Slider curvSlider = new Slider(0.01, 0.5, 0.1);
        curvSlider.setShowTickMarks(true);
        curvSlider.setShowTickLabels(true);
        curvSlider.setMajorTickUnit(0.1);
        curvSlider.setPrefWidth(250);

        TextField ransacIters = new TextField("500");
        ransacIters.setPrefWidth(60);

        TextField distThresh = new TextField("1.0");
        distThresh.setPrefWidth(60);

        TextField inlierRatio = new TextField("0.6");
        inlierRatio.setPrefWidth(60);

        CheckBox globfitCheckbox = new CheckBox("Enforce global constraints");
        globfitCheckbox.setSelected(true);
        globfitCheckbox.setStyle("-fx-text-fill: #BBBBBB;");

        Label resultLabel = new Label("Ready");
        resultLabel.setStyle("-fx-text-fill: #BBBBBB;");

        Button runBtn = new Button("Run Pipeline");
        runBtn.setDefaultButton(true);
        Button resetBtn = new Button("Reset");

        Runnable runPipeline = () -> {
            double angle = angleSlider.getValue();
            double curvThresh = curvSlider.getValue();
            int iters = parseIntSafe(ransacIters.getText(), 500);
            double dist = parseDoubleSafe(distThresh.getText(), 1.0);
            double minInlier = parseDoubleSafe(inlierRatio.getText(), 0.6);

            Platform.runLater(() -> resultLabel.setText("Running..."));

            List<FaceRegion> regions = regionGrower.grow(originalMesh, angle, curvThresh, Double.MAX_VALUE);
            log.info("Layer 1: {} regions", regions.size());

            List<PrimitiveFit> fits = ransacFitter.fitAll(originalMesh, regions, iters, dist, minInlier, 0);
            log.info("Layer 2: {} primitive fits", fits.size());

            List<GeometricRelationship> relationships = List.of();
            if (globfitCheckbox.isSelected()) {
                GlobFitRefiner.RefinementResult refined = globFitRefiner.refine(fits, 5.0, -1, 0.05);
                fits = refined.primitives();
                relationships = refined.relationships();
                log.info("Layer 3: {} geometric relationships enforced", relationships.size());
            }

            // Publish fits as face groups via context event
            List<FaceGroup> groups = new ArrayList<>();
            for (int i = 0; i < fits.size(); i++) {
                PrimitiveFit fit = fits.get(i);
                String name = fit.type().name() + " #" + i;
                groups.add(new FaceGroup(i, name, fit.inlierFaceIndices(), Color.GRAY));
            }

            // Publish event so the app can update scene tree and colorize
            context.publishEvent(new PrimitiveDetectionResult(groups, fits, relationships));

            final int regionCount = regions.size();
            final int fitCount = fits.size();
            final int relCount = relationships.size();
            Platform.runLater(() -> resultLabel.setText(
                    regionCount + " regions → " + fitCount + " primitives, " + relCount + " constraints"));
        };

        runBtn.setOnAction(e -> runPipeline.run());
        resetBtn.setOnAction(e -> {
            context.setCurrentMesh(originalMesh);
            resultLabel.setText("Ready");
        });

        String labelStyle = "-fx-text-fill: #BBBBBB;";

        HBox angleRow = new HBox(10, label("Angle:", labelStyle), angleSlider, angleField, label("°", labelStyle));
        angleRow.setAlignment(Pos.CENTER_LEFT);

        HBox curvRow = new HBox(10, label("Curvature:", labelStyle), curvSlider);
        curvRow.setAlignment(Pos.CENTER_LEFT);

        HBox ransacRow = new HBox(10, label("RANSAC iters:", labelStyle), ransacIters,
                label("Dist:", labelStyle), distThresh, label("Min inlier:", labelStyle), inlierRatio);
        ransacRow.setAlignment(Pos.CENTER_LEFT);

        HBox buttonRow = new HBox(10, runBtn, resetBtn);
        buttonRow.setAlignment(Pos.CENTER_RIGHT);

        VBox root = new VBox(10, angleRow, curvRow, ransacRow, globfitCheckbox, resultLabel, buttonRow);
        root.setPadding(new Insets(16));
        root.setStyle("-fx-background-color: #3C3F41;");

        Scene scene = new Scene(root);
        scene.setFill(Color.web("#3C3F41"));
        dialog.setScene(scene);
        dialog.setWidth(550);
        dialog.setHeight(280);
        dialog.setResizable(false);
        dialog.show();
    }

    private Label label(String text, String style) {
        Label l = new Label(text);
        l.setStyle(style);
        return l;
    }

    private int parseIntSafe(String s, int fallback) {
        try { return Integer.parseInt(s.trim()); } catch (NumberFormatException e) { return fallback; }
    }

    private double parseDoubleSafe(String s, double fallback) {
        try { return Double.parseDouble(s.trim()); } catch (NumberFormatException e) { return fallback; }
    }

    // ========================= Event record =========================

    /**
     * Published when the plugin completes primitive detection via the dialog.
     * The app can listen for this to update the scene tree and colorize the mesh.
     */
    public record PrimitiveDetectionResult(List<FaceGroup> groups, List<PrimitiveFit> fits, List<GeometricRelationship> relationships) {}

    // ========================= MeshOperation =========================

    private class PrimitiveDetectionOperation implements MeshOperation {
        @Override
        public String getName() { return "Primitive Detection"; }

        @Override
        public String getDescription() {
            return "Detects geometric primitives (planes, spheres, cylinders, cones) in mesh faces";
        }

        @Override
        public MeshData execute(MeshData input) {
            List<FaceRegion> regions = regionGrower.grow(input, 15.0, 0.1, Double.MAX_VALUE);
            List<PrimitiveFit> fits = ransacFitter.fitAll(input, regions, 500, 1.0, 0.6, 0);
            GlobFitRefiner.RefinementResult refined = globFitRefiner.refine(fits, 5.0, -1, 0.05);
            // Return input unchanged — the operation produces fits as a side effect
            return input;
        }
    }
}
