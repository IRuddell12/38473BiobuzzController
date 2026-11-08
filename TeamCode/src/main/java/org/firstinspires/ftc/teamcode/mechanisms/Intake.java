package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

    private DcMotorEx intake;
    private CRServo pollenTransfer;
    private DcMotor nectarAssist;
    private CRServo nectarTransfer;
    private Servo probe;
    private Servo pollenGate;
    private Servo nectarGate;
    private Servo intakeLift;

    double probeDown = .4;
    double probeUp = 1;

    double pollenGateUp = 0.6;
    double pollenGateDown = 1;
    double nectarGateUp = 0.4;
    double nectarGateDown = 0;

    // Ball tracking
    private int ballCount = 0;
    private final int maxBalls = 4;

    private final ElapsedTime detectionTimer = new ElapsedTime();

    private double currentThreshold = 0.4; // Calibrate on robot
    private double detectionDelay = 0.15;

    private boolean gamePieceDetected = false;
    private boolean detecting = false;
    private boolean detectionArmed = true;

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        nectarAssist = hwMap.get(DcMotor.class, "nectar_assist");
        nectarAssist.setDirection(DcMotorSimple.Direction.REVERSE);

        pollenTransfer = hwMap.get(CRServo.class, "pollen_transfer");
        nectarTransfer = hwMap.get(CRServo.class, "nectar_transfer");
        nectarTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        pollenGate = hwMap.get(Servo.class, "pollen_gate");
        nectarGate = hwMap.get(Servo.class, "nectar_gate");

        pollenGate.setPosition(pollenGateDown);
        nectarGate.setPosition(nectarGateDown);

        intakeLift = hwMap.get(Servo.class, "intake_lift");
        intakeLift.scaleRange(0, 0.6);

        probe = hwMap.get(Servo.class, "probe");
        probe.scaleRange(probeDown, probeUp);
        probe.setPosition(probeUp);
    }

    public void runIntake(double power) {
        intake.setPower(power);
    }

    public double intakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public void runTransfers(double power) {
        nectarAssist.setPower(power);
        pollenTransfer.setPower(power);
        nectarTransfer.setPower(power * 0.7);
    }

    public void setIntakeLift(double position) {
        intakeLift.setPosition(position);
    }

    public double getIntakeLiftPos() {
        return intakeLift.getPosition();
    }

    public void setProbe(int position) {
        if (position == 1) {
            probe.setPosition(probeDown);
        } else {
            probe.setPosition(probeUp);
        }
    }

    public double getProbePosition() {
        return probe.getPosition();
    }

    public void setGates(int position) {
        if (position == 1) {
            pollenGate.setPosition(pollenGateUp);
            nectarGate.setPosition(nectarGateUp);
        } else {
            pollenGate.setPosition(pollenGateDown);
            nectarGate.setPosition(nectarGateDown);
        }
    }

    public double getPollenGatePosition() {
        return pollenGate.getPosition();
    }

    public double getNectarGatePosition() {
        return nectarGate.getPosition();
    }

    // Call continuously in your OpMode loop or command execute()
    public void updateIntakeDetection() {
        double current = intake.getCurrent(CurrentUnit.AMPS);

        if (current > currentThreshold && detectionArmed) {
            if (!detecting) {
                detecting = true;
                detectionTimer.reset();
            }

            if (detectionTimer.seconds() >= detectionDelay) {
                ballCount++;

                gamePieceDetected = true;
                detectionArmed = false;
                detecting = false;
            }
        } else if (current <= currentThreshold) {
            detecting = false;
            detectionArmed = true;
            gamePieceDetected = false;
        }
    }

    public boolean isGamePieceDetected() {
        return gamePieceDetected;
    }

    public int getBallCount() {
        return ballCount;
    }

    public boolean isFull() {
        return ballCount >= maxBalls;
    }

    // Call after a successful shot or confirmed removal
    public void ballShot() {
        if (ballCount > 0) {
            ballCount--;
        }
    }

    public void preventOverfill() {
        if (ballCount > 4) {
            runIntake(-.5); // Reverse intake
        } else {
            runIntake(1.0);
        }
    }

    public void resetBallCount() {
        ballCount = 0;
        gamePieceDetected = false;
        detecting = false;
        detectionArmed = true;
    }
}