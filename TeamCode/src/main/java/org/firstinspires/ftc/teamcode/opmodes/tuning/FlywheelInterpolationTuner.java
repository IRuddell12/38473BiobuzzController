package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Interpolation Tuner")
public class FlywheelInterpolationTuner extends OpMode {

    Shooter shooter = new Shooter();
    Hood hood = new Hood();
    Intake intake = new Intake();
    private Follower follower;

    double gateClosed = 0.15;
    double gateOpen = 0.8;
    double flywheelSpeed = 1200;//
    double intakeSpeed = 0.8;

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(72, 72, 90);

    @Override
    public void init() {
        shooter.init(hardwareMap);
        hood.init(hardwareMap);
        intake.init(hardwareMap);

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);
    }

    double hoodStep = 1.0;

    @Override
    public void loop() {

        follower.update();
        Pose robotPose = follower.pose();


        if (gamepad1.b) {
            shooter.setGatePosition(gateOpen);
        } else {
            shooter.setGatePosition(gateClosed);
        }

        if (gamepad1.rightBumperWasPressed() && intakeSpeed < 1.0) {
            intakeSpeed += 0.05;
        }

        if (gamepad1.left_bumper && intakeSpeed < 1.0) {
            intakeSpeed -=0.05;
        }

        if (gamepad1.right_trigger > 0.3) {
            intake.runIntake(intakeSpeed);
        } else {
            intake.runIntake(0);
        }

        if (gamepad1.dpadUpWasPressed() && hoodStep > 0) {
            hoodStep -= 0.05;
        }

        if (gamepad1.dpadDownWasPressed() && hoodStep < 1) {
            hoodStep +=0.05;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            flywheelSpeed -= 10;
        }

        if (gamepad1.dpadRightWasPressed()) {
            flywheelSpeed += 10;
        }

        double robotX = robotPose.x();
        double robotY = robotPose.y();

        double xGoal = 84;
        double yGoal = 72;

        double deltaX = xGoal - robotX;
        double deltaY = yGoal - robotY;

        //double targetHeading = Math.atan2(deltaY, deltaX);
        double targetDistance = Math.hypot(deltaX,deltaY);

        hood.setHoodPos(hoodStep);
        shooter.setFlywheelSpeed(flywheelSpeed);

        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Hood Posistion", hoodStep);
        telemetry.addData("Flywheel Speed", flywheelSpeed);
        telemetry.addData("Distance", targetDistance);
    }
}

