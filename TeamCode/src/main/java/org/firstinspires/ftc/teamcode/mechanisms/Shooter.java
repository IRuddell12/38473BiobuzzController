package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx flywheel;

    public void init(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "flywheel1");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
