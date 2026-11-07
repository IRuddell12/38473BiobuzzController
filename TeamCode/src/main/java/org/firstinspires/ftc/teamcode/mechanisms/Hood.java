package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private Servo pollenHood;
    private Servo nectarHood;
    private double pollenCurrentPos = 0.2;
    private double nectarCurrentPos = 0.0;
    double pollenMinRange = 0; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION UP!!!
    double pollenMaxRange = .7; //SET THIS VALUE FOR THE FARTHEST RANGE OF MOTION DOWN!!!
    double nectarMinRange = 1;
    double nectarMaxRange = 0.35;


    public void init(HardwareMap hwMap) {
        pollenHood = hwMap.get(Servo.class, "pollen_hood");
        pollenHood.scaleRange(pollenMinRange, pollenMaxRange);
        pollenHood.setPosition(pollenMinRange);

        nectarHood = hwMap.get(Servo.class, "nectar_hood");
        nectarHood.scaleRange(nectarMaxRange, nectarMinRange);
        nectarHood.setPosition(nectarMinRange);
    }

    public void setPollenHoodPos(double pos) {
        pollenCurrentPos = pos;
        pollenHood.setPosition(pos);
    }

    public void setNectarHoodPos(double pos) {
        nectarCurrentPos = pos;
        nectarHood.setPosition(pos);
    }
}

