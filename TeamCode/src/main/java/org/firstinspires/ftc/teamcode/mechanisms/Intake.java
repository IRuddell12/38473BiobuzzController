package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx intake;
    private CRServo feeder;

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        feeder = hwMap.get(CRServo.class, "feeder");
        feeder.setDirection(CRServo.Direction.REVERSE);
    }

    public void runIntake(double power) {
        intake.setPower(power);
    }

    public void runFeeder(double power) {
        feeder.setPower(power);
    }
}