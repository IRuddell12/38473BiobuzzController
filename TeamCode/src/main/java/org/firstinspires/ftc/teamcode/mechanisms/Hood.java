package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hood {
    private Servo hood;
    private double currentPos = 0.2;
    double minRange = 0.05; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION UP!!!
    double maxRange = 0.95 ; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION DOWN!!!

    public void init(HardwareMap hwMap) {
        hood = hwMap.get(Servo.class, "hood_servo");
        hood.scaleRange(minRange, maxRange);
    }

    public void setHoodPos(double pos) {
        currentPos = pos;
        hood.setPosition(pos);
    }

    public double getHoodPos() {
        return currentPos;// return last set position
    }
}

