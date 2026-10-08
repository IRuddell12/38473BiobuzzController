package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

//@Disabled
@TeleOp(name = "Flywheel Constants Tuner")

public class FlywheelPFTuner extends OpMode {

    Intake intake = new Intake();
    public DcMotorEx flywheelMotor1;
    //public DcMotorEx flywheelMotor2;

    public double highVelocity = 1650;
    public double lowVelocity = 1300;
    double curTargetVelocity = highVelocity;

    double F = 13; //13
    double P = 150; //150

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        intake.init(hardwareMap);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel_motor1");
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel_motor2");
//        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);
//        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        intake.runIntake(0.1);

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor1.setVelocity(curTargetVelocity);

        //flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //flywheelMotor2.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor1.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.3f", curVelocity);
        telemetry.addData("Error", "%.3f", error);
        telemetry.addLine("------------------------------");
        telemetry.addData("tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad U/D)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}

