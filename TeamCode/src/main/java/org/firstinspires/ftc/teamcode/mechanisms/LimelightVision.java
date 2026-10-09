
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class LimelightVision {

    private static final int RED_TAG_MIN = 30;
    private static final int RED_TAG_MAX = 37;

    // Maximum horizontal aiming error, in degrees.
    private static final double ALIGN_TOLERANCE_DEG = 1.5;

    private Limelight3A limelight;

    private boolean redTagVisible = false;
    private int targetId = -1;

    private double targetX = Double.NaN;
    private double targetY = Double.NaN;
    private double targetRoll = Double.NaN;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set this to the pipeline configured for your AprilTags.
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void update() {
        // Clear each frame so old detections aren't reused.
        redTagVisible = false;
        targetId = -1;
        targetX = Double.NaN;
        targetY = Double.NaN;
        targetRoll = Double.NaN;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();

            if (id < RED_TAG_MIN || id > RED_TAG_MAX) {
                continue;
            }

            redTagVisible = true;
            targetId = id;
            targetX = tag.getTargetXDegrees();
            targetY = tag.getTargetYDegrees();

            targetRoll = tag.getTargetPoseCameraSpace()
                    .getOrientation()
                    .getRoll(AngleUnit.DEGREES);

            // First matching red tag. If several are visible,
            // target selection should be improved for your setup.
            return;
        }
    }

    public boolean seesRedTag() {
        return redTagVisible;
    }

    public int getTargetId() {
        return targetId;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public double getTargetRoll() {
        return targetRoll;
    }

    public boolean isAligned() {
        return redTagVisible
                && Double.isFinite(targetX)
                && Math.abs(targetX) <= ALIGN_TOLERANCE_DEG;
    }

    /**
     * Provisional roll-based check.
     * Validate the Limelight roll convention against actual
     * scorable and unscorable CELL orientations before use.
     */
    public boolean isScorable() {
        return redTagVisible
                && Double.isFinite(targetRoll)
                && Math.abs(targetRoll) < 90.0;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public void start() {
        if (limelight == null) {
            limelight.start();
        }
    }
}