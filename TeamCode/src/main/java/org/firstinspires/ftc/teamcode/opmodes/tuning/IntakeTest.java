package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp (name = "Intake Test")
public class IntakeTest extends OpMode {

    private DcMotorEx intake;
    private DcMotorEx flywheel;
    private CRServo leftIntake;
    private CRServo rightIntake;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake_motor");

        flywheel = hardwareMap.get(DcMotorEx.class, "nectar_flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftIntake = hardwareMap.get(CRServo.class,"left_intake_servo");

        //rightIntake = hardwareMap.get(CRServo.class,"right_intake_servo");
        //rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.3){
            intake.setPower(1.0);
            //leftIntake.setPower(1.0);
            //rightIntake.setPower(1.0);
        } else {
            intake.setPower(0);
            //rightIntake.setPower(0);
            //leftIntake.setPower(0);
        }

        if (gamepad1.left_trigger > 0.3) {
            flywheel.setVelocity(800);
        } else {
            flywheel.setVelocity(0);
        }
    }
}
