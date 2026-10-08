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


    @Override
    public void init() {
        //-----------SETS STARTING POSE-------------
        follower = Constants.create(hardwareMap);
        intake.init(hardwareMap);
        //hood.init(hardwareMap);
        flywheel.init(hardwareMap);
        automatedDrive = false;

        /*TODO
        if (startingPoseAuto != null) {
            follower.setStartingPose(startingPoseBlue);
        } else {
            follower.setStartingPose(manualStart);
        }*/

        flywheel.setGatePosition(0.5);

        //telemetry.addData("Initialization Complete, Robot Pose after Auto = ", startingPoseBlue);
        telemetry.addData("Initialization Complete, Robot Pose = ", manualStartPose);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        //------------------DRIVER CODE-------------------
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers);


        follower.update();
        Pose robotPose = follower.pose();

        //---------ROBOT TARGET HEADING CODE (SUBSET OF DRIVER CODE)---------
        //Calculates heading of robot based on Goal and adjusts
        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double heading = robotPose.heading();

        double xGoal = 84;
        double yGoal;

        if (robotY > 72) {
            yGoal = 84; //TODO Should stay the same but feel free to tune if needed
        } else {
            yGoal = 60; //TODO Should stay the same but feel free to tune if needed
        }

        double deltaX = xGoal - robotX;
        double deltaY = yGoal - robotY;

        double targetDistance = Math.sqrt((deltaX*deltaX)+(deltaY*deltaY));

        double targetHeading = Math.atan2(deltaY, deltaX) + targetAngleOffset;

        double headingError = Math.atan2(Math.sin(targetHeading - heading),
                Math.cos(targetHeading - heading));

        double rotatePower = kP * headingError;

//        if (gamepad1.dpadRightWasPressed() && kP < 10.0) {
//            kP += 0.1;
//        } else if (gamepad1.dpadLeftWasPressed() && kP > 0.0) {
//            kP -= 0.1;
//        }

        // Clamp rotate power to max motor speed
        rotatePower = Math.max(-1.0, Math.min(1.0, rotatePower));

        double slowDrive = 0.25;

        //ONLY DRIVES IF NOT FOLLOWING AUTOMATED PATH
//        if (!automatedDrive) {
//            if (gamepad1.right_trigger > 0.3) {
//                follower.setTeleOpDrive(forward*slowDrive,strafe*slowDrive,rotate*slowDrive, false);
//            } else if (gamepad1.right_bumper) {
//                follower.setTeleOpDrive(forward,strafe,rotate, true);
//            } else if (gamepad1.left_trigger > 0.3) { // trigger held → auto-align
//                follower.setTeleOpDrive(forward, strafe, rotatePower, false);
//            } else {
//                follower.setTeleOpDrive(forward, strafe, rotate, false);
//            }
//        }

        //------------------LAUNCHER CODE---------------------
//        flywheel.setFlywheelSpeed(Shooter.flywheelInterpolated(targetDistance)+flywheelOffset);
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

        /*if (gamepad1.b) {
            intake.runIntake(Shooter.intakeInterpolated(targetDistance));
            flywheel.setGatePosition(0.5);
        } else if (gamepad2.right_trigger > 0.3){
            intake.runIntake(1.0);
        } else {
            flywheel.setGatePosition(0.15);
            intake.runIntake(0.0);
        }*/

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
