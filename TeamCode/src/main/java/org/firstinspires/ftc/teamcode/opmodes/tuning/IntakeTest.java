package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;


@TeleOp (name = "Intake Test")
public class IntakeTest extends OpMode {

    Shooter shooter = new Shooter();
    double intakeLiftUp = 0;
    double intakeLiftDown = 0.52;
    int probeUp = 0;
    int probeDown = 1;

    Intake intake = new Intake();
    @Override
    public void init() {
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
    }

    @Override
    public void loop() {
        shooter.setPollenFlywheelSpeed(1400);
        shooter.setNectarFlywheelSpeed(1600);

        if (gamepad1.right_trigger > 0.3) {
            intake.runIntake(1.0);
            intake.setIntakeLift(intakeLiftDown);
        } else {
            intake.runIntake(0.1);
            intake.runTransfers(1.0);
            intake.setIntakeLift(0); // 0 is to the left all the way, 1 is all the way to the right
        }

        if (gamepad1.left_trigger > 0.3) {
            intake.setProbe(probeDown);
        } else {
            intake.setProbe(probeUp);
        }

        if (gamepad1.a) {
            intake.setGates(1);
        } else {
            intake.setGates(0);
        }

        telemetry.addData("Intake Voltage", intake.intakeCurrent());
        telemetry.addLine();
        telemetry.addData("Intake Lift Position: ", intake.getIntakeLiftPos());
        telemetry.addData("Probe Position: ", intake.getProbePosition());
        telemetry.addData("Pollen Gate Position: ", intake.getPollenGatePosition());
        telemetry.addData("Nectar Gate Position", intake.getNectarGatePosition());
    }
}
