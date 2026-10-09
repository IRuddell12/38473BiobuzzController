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
    private double gateCloseAngle = 0;
    private double gateOpenAngle = 0.5;

    //---------FLYWHEEL CONSTANTS----------
    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double targetShootingVelocity = 1900;
    private double restingFlywheelVelocity = 1000;
    private double flywheelMaxSpinupTime = .25;

    //---------THROUGHPUT CONSTANTS----------
    private double shotFeedTime = .5; // tune this


    public void init(HardwareMap hwMap) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
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
                gate.setPosition(gateCloseAngle);
                flywheel.setVelocity(targetShootingVelocity); //set velocity

                if (shotsRemaining > 0) {
                    flywheel.setVelocity(targetShootingVelocity);
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                //set velocity
                flywheel.setVelocity(targetShootingVelocity);
                intake.runIntake(1.0);

                if (flywheelVelocity > (targetShootingVelocity * .95) || (stateTimer.seconds() > flywheelMaxSpinupTime)) {
                    intake.runIntake(1.0);
                    stateTimer.reset();
                    flywheelState = flywheelState.FEEDING;
                }
                break;


            case FEEDING:
                gate.setPosition(gateOpenAngle);

                if (shotsRemaining > 0) {
                    intake.runIntake(1.0);
                    intake.runFeeder(1);

                    if (stateTimer.seconds() > shotFeedTime) {
                        shotsRemaining--;
                        stateTimer.reset();

                        if (shotsRemaining > 0) {
                            flywheelState = FlywheelState.SPIN_UP;
                        } else {
                            flywheelState = FlywheelState.SHUTDOWN;
                        }
                    }
                }
                break;

            case SHUTDOWN:
                gate.setPosition(gateCloseAngle);
                intake.runIntake(0.5);
                intake.runFeeder(0);

                if (stateTimer.seconds() > 0.2) {
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
