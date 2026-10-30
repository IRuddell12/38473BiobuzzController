package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Interpolation Tuner", group = "Tuning")
public class FlywheelInterpolationTuner extends OpMode {

    Shooter shooter = new Shooter();
    Hood hood = new Hood();
    Intake intake = new Intake();
    //private Follower follower;

    double gateClosed = 0.15;
    double gateOpen = 0.8;
    double pollenFlywheelSpeed = 700;//
    double nectarFlywheelSpeed = 900;
    double intakePower = 1;

    //private final PoseFactory p = PoseFactory.degrees();

    //private final Pose startPose = p.of(72, 72, 90);

    @Override
    public void init() {
        shooter.init(hardwareMap);
        hood.init(hardwareMap);
        intake.init(hardwareMap);

        //follower = Constants.create(hardwareMap);

        //follower.setPose(startPose);
    }

    double pollenHoodStep = 0;
    double nectarHoodStep = 1;

    @Override
    public void loop() {

        //follower.update();
        //Pose robotPose = follower.pose();


        if (gamepad1.b) {
            shooter.setGatePosition(gateOpen);
        } else {
            shooter.setGatePosition(gateClosed);
        }

//        if (gamepad1.rightBumperWasPressed() && intakePower < 1.0) {
//            intakePower += 0.05;
//        }

//        if (gamepad1.left_bumper && intakePower > 0.0) {
//            intakePower -=0.05;
//        }

        if (gamepad1.right_trigger > 0.3) {
            intake.runIntake(intakePower);
        } else {
            intake.runIntake(0);
        }

        if (gamepad1.dpadUpWasPressed() && pollenHoodStep > 0) {
            pollenHoodStep -= 0.05;
        }

        if (gamepad1.dpadDownWasPressed() && pollenHoodStep < 1) {
            pollenHoodStep +=0.05;
        }

        if (gamepad1.yWasPressed() && nectarHoodStep < 1) {
            nectarHoodStep +=0.05;
        }

        if (gamepad1.aWasPressed() && nectarHoodStep > 0) {
            nectarHoodStep +=0.05;
        }


        if (gamepad1.dpadLeftWasPressed()) {
            pollenFlywheelSpeed -= 10;
        }

        if (gamepad1.dpadRightWasPressed()) {
            pollenFlywheelSpeed += 10;
        }

        if (gamepad1.x) {
            nectarFlywheelSpeed -= 10;
        }

        if (gamepad1.b) {
            nectarFlywheelSpeed += 10;
        }

        /*double robotX = robotPose.x();
        double robotY = robotPose.y();

        double xGoal = 84;
        double yGoal = 72;

        double deltaX = xGoal - robotX;
        double deltaY = yGoal - robotY;

        double targetHeading = Math.atan2(deltaY, deltaX);
        double targetDistance = Math.hypot(deltaX,deltaY); */

        hood.setNectarHoodPos(nectarHoodStep);
        hood.setPollenHoodPos(pollenHoodStep);
        shooter.setPollenFlywheelSpeed(pollenFlywheelSpeed);
        shooter.setNectarFlywheelSpeed(pollenFlywheelSpeed);

        //telemetry.addData("Intake Speed", intakePower);
        telemetry.addData("Pollen Hood Position:", pollenHoodStep);
        telemetry.addData("Nectar Hood Position:", nectarHoodStep);
        telemetry.addLine();
        telemetry.addData("Pollen Flywheel Speed", pollenFlywheelSpeed);
        telemetry.addData("Pollen Flywheel Current", shooter.getAmperagePollen());
        telemetry.addLine();
        telemetry.addData("Nectar Flywheel Speed", nectarFlywheelSpeed);
        telemetry.addData("Nectar FLywheel Current", shooter.getAmperageNectar());
//        telemetry.addData("Distance", targetDistance);
//        telemetry.addData("Target Heading", targetHeading);
    }
}

