package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Interpolation Tuner", group = "Tuning")
public class FlywheelInterpolationTuner extends OpMode {

    Shooter shooter = new Shooter();
    Hood hood = new Hood();
    Intake intake = new Intake();

    private Follower follower;

    int gateClosed = 0;
    int gateOpen = 1;
    double pollenFlywheelSpeed = 1250;//
    double nectarFlywheelSpeed = 1300;

    double intakePower = 1;

    double pollenHoodPos = 1;
    double nectarHoodPos = 0;

    double intakeLiftUp = 0;
    double intakeLiftDown = 0.52;

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(57.58, 5.80, 90);

    @Override
    public void init() {
        shooter.init(hardwareMap);
        hood.init(hardwareMap);
        intake.init(hardwareMap);


        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        Pose robotPose = follower.pose();

        if (gamepad1.right_bumper) {
            intake.setGates(gateOpen);
        } else {
            intake.setGates(gateClosed);
        }

        if (gamepad1.right_trigger > 0.3) {
            intake.runIntake(intakePower);
        } else {
            intake.runIntake(0);
            intake.runTransfers(0.7);
            intake.setIntakeLift(intakeLiftDown);
        }

        // Pollen hood
        if (gamepad1.dpadUpWasPressed()) {
            pollenHoodPos = Range.clip(pollenHoodPos + 0.05, 0, 1);
        }

        if (gamepad1.dpadDownWasPressed()) {
            pollenHoodPos = Range.clip(pollenHoodPos - 0.05, 0, 1);
        }

// Nectar hood
        if (gamepad1.triangleWasPressed()) {
            nectarHoodPos = Range.clip(nectarHoodPos - 0.05, 0, 1);
        }

        if (gamepad1.crossWasPressed()) {
            nectarHoodPos = Range.clip(nectarHoodPos + 0.05, 0, 1);
        }

        if (gamepad1.dpadLeftWasPressed()) {
            pollenFlywheelSpeed -= 10;
        }

        if (gamepad1.dpadRightWasPressed()) {
            pollenFlywheelSpeed += 10;
        }

        if (gamepad1.squareWasPressed()) {
            nectarFlywheelSpeed -= 10;
        }

        if (gamepad1.circleWasPressed()) {
            nectarFlywheelSpeed += 10;
        }

        double robotX = robotPose.x();
        double robotY = robotPose.y();

        double xGoal = 58;
        double yGoal = 54;

        double deltaX = xGoal - robotX;
        double deltaY = yGoal - robotY;

        double targetHeading = Math.atan2(deltaY, deltaX);
        double targetDistance = Math.hypot(deltaX,deltaY);

        hood.setNectarHoodPos(nectarHoodPos);
        hood.setPollenHoodPos(pollenHoodPos);
        shooter.setPollenFlywheelSpeed(pollenFlywheelSpeed);
        shooter.setNectarFlywheelSpeed(nectarFlywheelSpeed);

        telemetry.addLine("dpad up and down for pollen, triangle and cross for nectar");
        telemetry.addData("Pollen Hood Position:", pollenHoodPos);
        telemetry.addData("Nectar Hood Position:", nectarHoodPos);
        telemetry.addLine();
        telemetry.addLine("dpad right and left for pollen, square and circle for nectar");
        telemetry.addData("Pollen Flywheel Speed", pollenFlywheelSpeed);
        telemetry.addData("Pollen Flywheel Current", shooter.getAmperagePollen());
        telemetry.addLine();
        telemetry.addData("Nectar Flywheel Speed", nectarFlywheelSpeed);
        telemetry.addData("Nectar FLywheel Current", shooter.getAmperageNectar());
        telemetry.addLine();
        telemetry.addData("Distance", targetDistance);
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addLine();
        telemetry.addData("robot x: ", robotX);
        telemetry.addData("robot y: ", robotY);
    }
}

