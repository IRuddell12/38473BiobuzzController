package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

    private DcMotorEx intake;
    private CRServo feeder;
    private Servo probe;

    private double currentPos = 0.2;

    double down = 0.05;
    double up = 0.95;

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
        intake = hwMap.get(DcMotorEx.class, "intake_motor");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //probe = hwMap.get(Servo.class, "probe");
        //probe.scaleRange(down, up);
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

    public double getProbePos() {
        return currentPos;
    }
}