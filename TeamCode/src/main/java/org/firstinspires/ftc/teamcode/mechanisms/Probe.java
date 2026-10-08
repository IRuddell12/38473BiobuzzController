package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Probe {
    private Servo probe;
    private double currentPos = 0.2;

    double down = 0.05; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION UP!!!
    double up = 0.95 ; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION DOWN!!!

    public void init(HardwareMap hwMap) {
        probe = hwMap.get(Servo.class, "probe");
        probe.scaleRange(down, up);
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
