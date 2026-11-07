package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.PoseHolder;

//@Disabled
@TeleOp(name = "Tele-Op")
public class BiobuzzTeleOp extends OpMode {
    Intake intake = new Intake();
    Hood hood = new Hood();
    private Shooter shooter = new Shooter();
    private Follower follower;
    private  final PoseFactory p = PoseFactory.degrees();

    private final Pose manualStartPose = p.of(57.58, 5.80, 90);

    double flywheelOffset = 0;
    double targetAngleOffset = 0;
    double kP = 0.5; // proportional constant, tune this

    int gateOpen = 1;
    int gateClose = 0;

    int probeUp = 0;
    int probeDown = 1;


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        if (PoseHolder.savedPose != null) {
            follower.setPose(PoseHolder.savedPose);
        } else {
            follower.setPose(manualStartPose);
        }

        intake.init(hardwareMap);

        shooter.init(hardwareMap);
        hood.init(hardwareMap);
        intake.setIntakeLift(0);
        intake.setProbe(probeUp);

        intake.setGates(gateClose);

        telemetry.update();

        telemetry.addData("Starting Pose", PoseHolder.savedPose != null ? "Restored from Autonomous" : "Manual Start Pose");
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        //intake.updateIntakeDetection();
        //intake.preventOverfill();

        follower.update();
        Pose robotPose = follower.pose();


        //---------ROBOT TARGET HEADING CODE (SUBSET OF DRIVER CODE)---------
        //Calculates heading of robot based on Goal and adjusts
        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double heading = robotPose.heading();

// Select the goal based on the robot's equivalent Red-side position.
        double hiveX = 60;
        double hiveY;

        if (robotY < 70.75) {
            hiveY = 52;
        } else {
            hiveY = 88;
        }

        double deltaX = hiveX - robotX;
        double deltaY = hiveY - robotY;

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

        if (gamepad2.dpadRightWasPressed() && kP < 10.0) {
            kP += 0.1;
        } else if (gamepad2.dpadLeftWasPressed() && kP > 0.0) {
            kP -= 0.1;
        }

//------------------- DRIVER CODE -------------------
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        double slowDrive = 0.25;

        DrivePowers powers;

        if (gamepad1.right_trigger > 0) {
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
        shooter.setPollenFlywheelSpeed(Shooter.pollenFlywheelInterpolated(targetDistance)+flywheelOffset);
        shooter.setNectarFlywheelSpeed(Shooter.nectarFlywheelInterpolated(targetDistance)+flywheelOffset);

        hood.setPollenHoodPos(Shooter.pollenHoodInterpolated(targetDistance));
        hood.setNectarHoodPos(Shooter.nectarHoodInterpolated(targetDistance));

        if (gamepad2.dpadUpWasPressed() && flywheelOffset < 100) {
            flywheelOffset += 10;
        } else if (gamepad2.dpadDownWasPressed() &&  flywheelOffset > 0) {
            flywheelOffset -= 10;
        }

        intake.runTransfers(1.0);
        intake.runIntake(1.0);
        intake.setIntakeLift(0.55);

        if (gamepad1.circle) {
            intake.setGates(gateOpen);
        } else {
            intake.setGates(gateClose);
        }

        if (gamepad1.right_bumper) {
            intake.setProbe(probeDown);
        } else {
            intake.setProbe(probeUp);
        }

        telemetry.addData("Distance to Goal", targetDistance);
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("X Position", robotX);
        telemetry.addData("Y position", robotY);
        telemetry.addData("Current Heading", Math.toDegrees(heading));
        telemetry.addData("Flywheel Offset", flywheelOffset);
        telemetry.addData("kP Value", kP);
        telemetry.addData("Target Offset", targetAngleOffset);
    }

    @Override
    public void stop() {
        PoseHolder.savedPose = null;
    }
}
