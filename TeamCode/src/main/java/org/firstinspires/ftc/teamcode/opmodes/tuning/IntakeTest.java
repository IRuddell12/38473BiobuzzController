package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "Intake Test")
public class IntakeTest extends OpMode {

    private DcMotorEx intake;
    private DcMotorEx flywheel;
    private Servo intakeLift;
    private CRServo nectarAssist;
    private CRServo pollenTransfer;
    private CRServo nectarTransfer;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intakeLift = hardwareMap.get(Servo.class, "intake_lift");

        nectarAssist = hardwareMap.get(CRServo.class, "nectar_assist");

        pollenTransfer = hardwareMap.get(CRServo.class, "pollen_transfer");

        nectarTransfer = hardwareMap.get(CRServo.class, "nectar_transfer");

        intakeLift.setPosition(0);

        //flywheel = hardwareMap.get(DcMotorEx.class, "nectar_flywheel");
       //flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftIntake = hardwareMap.get(CRServo.class,"left_intake_servo");

        //rightIntake = hardwareMap.get(CRServo.class,"right_intake_servo");
        //rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.3) {
            intake.setPower(1.0);
            nectarTransfer.setPower(-1.0);
            pollenTransfer.setPower(1.0);
            nectarAssist.setPower(1.0);
            intakeLift.setPosition(.3);
        } else {
            intake.setPower(0);
            nectarTransfer.setPower(0);
            pollenTransfer.setPower(0);
            nectarAssist.setPower(0);
            intakeLift.setPosition(0);
        }

        if (gamepad1.left_trigger > 0.3) {
            //flywheel.setVelocity(800);
        } else {
            //flywheel.setVelocity(0);
        }
    }
}
