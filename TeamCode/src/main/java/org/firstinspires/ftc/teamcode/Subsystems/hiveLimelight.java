package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

/**
 * Uses a Limelight 3A (AprilTag/fiducial pipeline) to
 *  1) work out which end of our HIVE beam is tipped up, and
 *  2) measure the distance from the camera to our HIVE's tags.
 *
 * Limelight setup (web UI at 172.29.0.1:5801):
 *  - Pipeline PIPELINE_INDEX = AprilTag/Fiducial, family 36h11, marker size 82.55 mm (3.25 in)
 *  - Robot config: Limelight 3A named "limelight"
 */
public class hiveLimelight {

    public enum UpCell { HIGH_Y_END, LOW_Y_END, UNKNOWN }

    // ---- Tune / verify these ----
    public static String DEVICE_NAME = "limelight";
    public static int PIPELINE_INDEX = 0;

    // HIVE tag IDs, confirmed against the Competition Manual (Fig 9-17, tag family 36h11).
    //   Red:  30-33 opposite-audience CELL, 34-37 audience CELL
    //   Blue: 38-41 audience CELL,          42-45 opposite-audience CELL
    public static final int RED_OPPOSITE_MIN = 30, RED_OPPOSITE_MAX = 33;
    public static final int RED_AUDIENCE_MIN = 34, RED_AUDIENCE_MAX = 37;
    public static final int BLUE_AUDIENCE_MIN = 38, BLUE_AUDIENCE_MAX = 41;
    public static final int BLUE_OPPOSITE_MIN = 42, BLUE_OPPOSITE_MAX = 45;

    // Is the audience side the +Y end in Pedro coordinates? Confirmed: the audience side is low Y.
    public static boolean AUDIENCE_IS_HIGH_Y = false;

    // ASSUMPTION: the tags we can see belong to the LOW (tipped down) CELL, so the UP CELL is the other end.
    // Test it (see getVisibleIdsString) and flip this if it's backwards.
    public static boolean VISIBLE_CELL_IS_LOW = true;

    private final Limelight3A limelight;
    private turretAim.Alliance alliance;

    private boolean seesHighY = false;
    private boolean seesLowY = false;
    private double distanceInches = Double.NaN;
    private final List<Integer> visibleIds = new ArrayList<>();

    public hiveLimelight(HardwareMap hardwareMap, turretAim.Alliance alliance) {
        this.alliance = alliance;
        limelight = hardwareMap.get(Limelight3A.class, DEVICE_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE_INDEX);
    }

    public void setAlliance(turretAim.Alliance alliance) {
        this.alliance = alliance;
    }

    /** Call once in start(). */
    public void start() {
        limelight.start();
    }

    /** Call once in stop(). */
    public void stop() {
        limelight.stop();
    }

    /** Call every loop. Reads the latest result and refreshes everything below. */
    public void update() {
        seesHighY = false;
        seesLowY = false;
        distanceInches = Double.NaN;
        visibleIds.clear();

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double distSum = 0;
        int distCount = 0;

        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            int id = fr.getFiducialId();
            boolean isAudience = isAudienceTag(id);
            boolean isOpposite = isOppositeTag(id);
            if (!isAudience && !isOpposite) continue; // not our HIVE

            visibleIds.add(id);
            boolean highY = (isAudience == AUDIENCE_IS_HIGH_Y);
            if (highY) seesHighY = true; else seesLowY = true;

            // Line-of-sight distance from the camera to this tag
            Pose3D pose = fr.getTargetPoseCameraSpace();
            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;
            double meters = Math.sqrt(x * x + y * y + z * z);
            distSum += DistanceUnit.METER.toInches(meters);
            distCount++;
        }

        if (distCount > 0) distanceInches = distSum / distCount;
    }

    /** True if we can currently see at least one tag on our HIVE. */
    public boolean hasTarget() {
        return !Double.isNaN(distanceInches);
    }

    /** Which end of our HIVE beam is tipped up. UNKNOWN if we see no tags or tags from both ends. */
    public UpCell getUpCell() {
        if (seesHighY == seesLowY) return UpCell.UNKNOWN; // neither or both
        boolean visibleIsHighY = seesHighY;
        boolean upIsHighY = VISIBLE_CELL_IS_LOW ? !visibleIsHighY : visibleIsHighY;
        return upIsHighY ? UpCell.HIGH_Y_END : UpCell.LOW_Y_END;
    }

    /** Camera-to-HIVE-tag distance in inches (average of visible tags), or NaN if none. */
    public double getDistanceInches() {
        return distanceInches;
    }

    /** For telemetry while verifying the tag ID mapping. */
    public String getVisibleIdsString() {
        return visibleIds.toString();
    }

    private boolean isAudienceTag(int id) {
        return alliance == turretAim.Alliance.RED
                ? (id >= RED_AUDIENCE_MIN && id <= RED_AUDIENCE_MAX)
                : (id >= BLUE_AUDIENCE_MIN && id <= BLUE_AUDIENCE_MAX);
    }

    private boolean isOppositeTag(int id) {
        return alliance == turretAim.Alliance.RED
                ? (id >= RED_OPPOSITE_MIN && id <= RED_OPPOSITE_MAX)
                : (id >= BLUE_OPPOSITE_MIN && id <= BLUE_OPPOSITE_MAX);
    }
}