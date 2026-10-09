package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx intake;
    private CRServo feeder;
    private Servo probe;

    private double currentPos = 0.2;

    double down = 0.05; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION UP!!!
    double up = 0.95 ; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION DOWN!!!

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        feeder = hwMap.get(CRServo.class, "feeder");
        feeder.setDirection(CRServo.Direction.REVERSE);

        probe = hwMap.get(Servo.class, "probe");
        probe.scaleRange(down, up);
    }

    public void runIntake(double power) {
        intake.setPower(power);
    }

    public void runFeeder(double power) {
        feeder.setPower(power);
    }

    public void setProbe(int position) {
        if (position == 1) {
            probe.setPosition(down);
            currentPos = down;
        } else {
            probe.setPosition(up);
            currentPos = up;
        }
    }


    public double getProbePos() {
        return currentPos;// return last set position
    }
}