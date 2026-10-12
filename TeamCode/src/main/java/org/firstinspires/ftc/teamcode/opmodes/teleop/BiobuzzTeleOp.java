package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.mechanisms.HoodAdjuster;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.PoseHolder;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

//@Disabled
@TeleOp(name = "Tele-Op")
public class BiobuzzTeleOp extends OpMode {
    Intake intake = new Intake();
    //HoodAdjuster hood = new HoodAdjuster();
    private Shooter flywheel = new Shooter();
    private Follower follower;
    //public static Pose startingPoseBlue;
    private  final PoseFactory p = PoseFactory.degrees();

    private final Pose manualStartPose = p.of(55.54, 8.11, 90);
    private boolean automatedDrive = false;

    double flywheelOffset = 0;
    double targetAngleOffset = 0;
    double kP = 0.4; // proportional constant, tune this

    private static final double FIELD_SIZE = 144.0;

    private Pose alliancePose(Pose redPose) {
        if (RobotSettings.allianceColor
                != RobotSettings.AllianceColor.BLUE) {
            return redPose;
        }

        double blueX = FIELD_SIZE - redPose.x();
        double blueY = FIELD_SIZE - redPose.y();

        // PoseFactory.degrees() expects degrees.
        double blueHeading =
                (Math.toDegrees(redPose.heading()) + 180.0) % 360.0;

        return p.of(blueX, blueY, blueHeading);
    }


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        if (PoseHolder.savedPose != null) {
            // Autonomous already saved the alliance-adjusted pose.
            follower.setPose(PoseHolder.savedPose);
        } else {
            // Transform the manual starting pose for Blue.
            follower.setPose(alliancePose(manualStartPose));
        }

        intake.init(hardwareMap);
        flywheel.init(hardwareMap);
        automatedDrive = false;

        flywheel.setGatePosition(0.5);

        telemetry.addData(
                "Selected Alliance",
                RobotSettings.allianceColor == null
                        ? "NOT SELECTED"
                        : RobotSettings.allianceColor
        );
        telemetry.update();

        telemetry.addData("Starting Pose", PoseHolder.savedPose != null ? "Restored from Autonomous" : "Manual Start Pose");
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        intake.updateIntakeDetection();
        intake.preventOverfill();

        follower.update();
        Pose robotPose = follower.pose();


        //---------ROBOT TARGET HEADING CODE (SUBSET OF DRIVER CODE)---------
        //Calculates heading of robot based on Goal and adjusts
        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double heading = robotPose.heading();

        // Canonical goal coordinates for Red.
        double redEquivalentY = robotY;

        if (RobotSettings.allianceColor
                == RobotSettings.AllianceColor.BLUE) {
            redEquivalentY = FIELD_SIZE - robotY;
        }

// Select the goal based on the robot's equivalent Red-side position.
        double redGoalX = 84;
        double redGoalY = redEquivalentY > 72 ? 84 : 60;

// Convert the selected goal to the active alliance.
        double xGoal = redGoalX;
        double yGoal = redGoalY;

        if (RobotSettings.allianceColor
                == RobotSettings.AllianceColor.BLUE) {
            xGoal = FIELD_SIZE - redGoalX;
            yGoal = FIELD_SIZE - redGoalY;
        }

        double deltaX = xGoal - robotX;
        double deltaY = yGoal - robotY;

        double targetDistance =
                Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        double targetHeading =
                Math.atan2(deltaY, deltaX) + targetAngleOffset;

        double headingError = Math.atan2(
                Math.sin(targetHeading - heading),
                Math.cos(targetHeading - heading)
        );

        double rotatePower = kP * headingError;
        // Clamp rotate power to max motor speed
        rotatePower = Math.max(-1.0, Math.min(1.0, rotatePower));

//        if (gamepad2.dpadRightWasPressed() && kP < 10.0) {
//            kP += 0.1;
//        } else if (gamepad2.dpadLeftWasPressed() && kP > 0.0) {
//            kP -= 0.1;
//        }

//------------------- DRIVER CODE -------------------

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double slowDrive = 0.25;

        DrivePowers powers;

        if (gamepad1.right_trigger > 0.3) {
            // Slow driving
            powers = ManualDrive.fieldCentric(
                    forward * slowDrive,
                    strafe * slowDrive,
                    rotate * slowDrive,
                    follower.pose().heading()
            );
        } else if (gamepad1.left_trigger > 0.3) {
            // Auto-align to the goal
            powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    rotatePower,
                    follower.pose().heading()
            );
        } else {
            // Normal field-centric driving
            powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    rotate,
                    follower.pose().heading()
            );
        }

        ManualDrive.driveOrHold(follower, powers);

        //------------------LAUNCHER CODE---------------------
        flywheel.setPollenFlywheelSpeed(Shooter.pollenFlywheelInterpolated(targetDistance)+flywheelOffset);
//        hood.setHoodPos(Shooter.hoodInterpolated(targetDistance));

        if (gamepad2.dpadUpWasPressed() && flywheelOffset < 100) {
            flywheelOffset += 10;
        } else if (gamepad2.dpadDownWasPressed() &&  flywheelOffset > 0) {
            flywheelOffset -= 10;
        }

        if (gamepad2.dpadLeftWasPressed() && targetAngleOffset > -25) {
            flywheelOffset -= 1.0;
        } else if (gamepad2.dpadDownWasPressed() &&  targetAngleOffset < 25) {
            flywheelOffset += 1.0;
        }

        if (gamepad1.b) {
            flywheel.setGatePosition(0.5);
        }

        telemetry.addData("Distance to Goal", targetDistance);
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("X Position", robotX);
        telemetry.addData("Y position", robotY);
        telemetry.addData("Current Heading", Math.toDegrees(heading));
        telemetry.addData("flywheelOffset", flywheelOffset);
        telemetry.addData("kP Value", kP);
        telemetry.addData("Target Offset", targetAngleOffset);
    }
}
