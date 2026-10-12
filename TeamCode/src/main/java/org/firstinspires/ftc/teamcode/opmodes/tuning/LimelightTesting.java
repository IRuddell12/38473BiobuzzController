package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;

@TeleOp(name = "Limelight Test", group = "Tuning")
public class LimelightTesting extends OpMode {

    private LimelightVision vision;

    @Override
    public void init() {
        vision = new LimelightVision();
        vision.init(hardwareMap);

        telemetry.addLine("WARRIOR DYNAMICS - LIMELIGHT TEST");
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();
    }

    @Override
    public void loop() {
        vision.update();

        boolean visible = vision.seesRedTag();

        telemetry.addData("Target Visible", visible);

        if (visible) {
            telemetry.addData("AprilTag ID", vision.getTargetId());
            telemetry.addData(
                    "Horizontal Offset (deg)",
                    "%.2f", vision.getTargetX()
            );
            telemetry.addData(
                    "Vertical Offset (deg)",
                    "%.2f", vision.getTargetY()
            );
            telemetry.addData(
                    "Target Roll (deg)",
                    "%.2f", vision.getTargetRoll()
            );

            telemetry.addData("Aligned", vision.isAligned());
            telemetry.addData(
                    "Scorable (current check)",
                    vision.isScorable()
            );
        } else {
            telemetry.addLine("No matching RED AprilTag detected.");
            telemetry.addData("Target ID", "None");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.stop();
        }
    }
}
