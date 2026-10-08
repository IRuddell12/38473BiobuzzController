package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Shooter {
    private DcMotorEx flywheel;
    Intake intake = new Intake();
    private Servo gate;
    double F = 13;
    double P = 150;

    private final ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        FEEDING,
        SHUTDOWN,
    }

    private FlywheelState flywheelState;

    //----------GATE CONSTANTS----------
    private double GATE_CLOSE_ANGLE = 0;
    private double GATE_OPEN_ANGLE = 0.5;

    //---------FLYWHEEL CONSTANTS----------
    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double targetShootingRPM = 1900;
    private double FLYWHEEL_MAX_SPINUP_TIME = .25;

    //---------THROUGHPUT CONSTANTS----------
    private double FEED_TIME_PER_SHOT = .2; // tune this


    public void init(HardwareMap hwMap) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        flywheel = hwMap.get(DcMotorEx.class, "left_flywheel_motor");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel.setVelocity(0.0);

        flywheelState = FlywheelState.IDLE;

        gate = hwMap.get(Servo.class, "gate_servo");
        gate.scaleRange(0.2,0.5);
        gate.setPosition(0.2);
    }

    public void update() {

        flywheelVelocity = flywheel.getVelocity();

        switch (flywheelState) {

            case IDLE:
                intake.runIntake(0.5);
                gate.setPosition(GATE_CLOSE_ANGLE);
                flywheel.setVelocity(targetShootingRPM); //set velocity

                if (shotsRemaining > 0) {
                    flywheel.setVelocity(targetShootingRPM);
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                //set velocity
                flywheel.setVelocity(targetShootingRPM);
                intake.runIntake(1.0);

                if (flywheelVelocity > (targetShootingRPM * .95) || (stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME)) {
                    intake.runIntake(1.0);
                    stateTimer.reset();
                    flywheelState = flywheelState.FEEDING;
                }
                break;

            case FEEDING:
                gate.setPosition(GATE_OPEN_ANGLE);

                // If more than 1 shot left, keep running loader and wait feedTime
                if (shotsRemaining > 0) {
                    intake.runIntake(1.0);
                    if (stateTimer.seconds() > FEED_TIME_PER_SHOT) {
                        shotsRemaining--;
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP; // next shot
                    }
                }
                // If this is the last shot, skip feedTime, go directly to LAST_SHOT
                else if (shotsRemaining <= 0) {
                    intake.runIntake(0.5); // stop loader for last shot
                    stateTimer.reset(); // reset timer for LAST_SHOT
                }
                break;

            case SHUTDOWN:
                intake.runIntake(0.5);

                if (stateTimer.seconds() > 0.2) {
                    flywheel.setVelocity(0); // idle speed
                    flywheelState = FlywheelState.IDLE;
                }
                break;
        }
    }

    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    public void launch() {
        gate.setPosition(0.5);
    }

    public void cancelShooting() {
        shotsRemaining = 0;
        flywheelState = FlywheelState.SHUTDOWN;
    }

    public boolean isBusy(){
        return flywheelState != flywheelState.IDLE;
    }

    //tells the motor to run at the inputted value with a broken down function: setFlywheelSpeed(x)
    public void setFlywheelSpeed(double speed) {
        flywheel.setVelocity(speed);
    }

    public boolean isDoneShooting() {
        return shotsRemaining == 0 && flywheelState == FlywheelState.IDLE;
    }

    public void setGatePosition(double position) {
        gate.setPosition(position);
    }
}
