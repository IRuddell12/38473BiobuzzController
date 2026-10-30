package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private Servo pollenHood;
    private Servo nectarHood;
    private double pollenCurrentPos = 0.2;
    private double nectarCurrentPos = 0.0;
    double minRange = 0.05; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION UP!!!
    double maxRange = 0.95 ; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION DOWN!!!

    public void init(HardwareMap hwMap) {
        pollenHood = hwMap.get(Servo.class, "pollen_hood");
        pollenHood.scaleRange(minRange, maxRange);

        nectarHood = hwMap.get(Servo.class, "nectar_hood");
        nectarHood.scaleRange(minRange, maxRange);
    }

    public void setPollenHoodPos(double pos) {
        pollenCurrentPos = pos;
        pollenHood.setPosition(pos);
    }

    public void setNectarHoodPos(double pos) {
        nectarCurrentPos = pos;
        nectarHood.setPosition(pos);
    }

    public double getNectarHoodPos() {
        return pollenCurrentPos;// return last set position
    }

    public double getPollenHoodPos() {
        return nectarCurrentPos;
    }
}

