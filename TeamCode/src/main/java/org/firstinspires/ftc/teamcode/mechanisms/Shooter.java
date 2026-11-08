package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Shooter {
    private DcMotorEx pollenFlywheel;
    private DcMotorEx nectarFlywheel;
    private Intake intake;
    Hood hood = new Hood();

    double pollenF = 13;
    double pollenP = 200;
    double nectarF = 13.95;
    double nectarP = 200;

    int gateOpen = 1;
    int gateClose = 0;

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
    private double pollenTargetVelocity = 1270;
    private double nectarTargetVelocity = 1300;
    private double flywheelMaxSpinupTime = .25;

    //---------THROUGHPUT CONSTANTS----------
    private double shotFeedTime = .75; // tune this


    public void init(HardwareMap hwMap) {
        PIDFCoefficients pollenPidfCoefficients = new PIDFCoefficients(pollenP, 0, 0, pollenF);
        pollenFlywheel = hwMap.get(DcMotorEx.class, "pollen_flywheel");
        pollenFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pollenPidfCoefficients);
        pollenFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenFlywheel.setVelocity(0.0);

        PIDFCoefficients nectarPidfCoefficients = new PIDFCoefficients(nectarP, 0, 0, nectarF);
        nectarFlywheel = hwMap.get(DcMotorEx.class, "nectar_flywheel");
        nectarFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, nectarPidfCoefficients);
        nectarFlywheel.setVelocity(0.0);

        hood = new Hood();
        hood.init(hwMap);

        flywheelState = FlywheelState.IDLE;
    }


    public void update() {

        double pollenVelocity = pollenFlywheel.getVelocity();
        double nectarVelocity = nectarFlywheel.getVelocity();

        switch (flywheelState) {

            case IDLE:
                intake.runIntake(0.5);
                intake.runTransfers(1.0);
                intake.setGates(gateClose);
                pollenFlywheel.setVelocity(pollenTargetVelocity);
                nectarFlywheel.setVelocity(nectarTargetVelocity);

                if (shotsRemaining > 0) {
                    pollenFlywheel.setVelocity(pollenTargetVelocity);
                    nectarFlywheel.setVelocity(nectarTargetVelocity);
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                // Set target velocities
                pollenFlywheel.setVelocity(pollenTargetVelocity);
                nectarFlywheel.setVelocity(nectarTargetVelocity);

                intake.runIntake(1.0);
                intake.runTransfers(1.0);

                // Check that BOTH flywheels reach 95% of target velocity
                boolean bothFlywheelsReady =
                        pollenVelocity >= (pollenTargetVelocity * 0.95) && nectarVelocity >= (nectarTargetVelocity * 0.95);

                if (bothFlywheelsReady ||
                        stateTimer.seconds() > flywheelMaxSpinupTime) {

                    intake.runIntake(1.0);
                    stateTimer.reset();
                    flywheelState = FlywheelState.FEEDING;
                }
                break;

            case FEEDING:
                intake.setGates(gateOpen);
                intake.runTransfers(1.0);

                if (shotsRemaining > 0) {
                    intake.runIntake(1.0);

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
                intake.setGates(gateClose);
                intake.runIntake(0.5);
                intake.runTransfers(1.0);

                if (stateTimer.seconds() > 0.2) {
                    flywheelState = FlywheelState.IDLE;
                }
                break;
        }
    }


    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE && numberOfShots > 0) {
            shotsRemaining = numberOfShots;
            stateTimer.reset();
        }
    }

    public void cancelShooting() {
        shotsRemaining = 0;
        flywheelState = FlywheelState.SHUTDOWN;
    }

    public boolean isBusy() {
        return flywheelState != flywheelState.IDLE;
    }

    //tells the motor to run at the inputted value with a broken down function: setFlywheelSpeed(x)
    public void setPollenFlywheelSpeed(double speed) {
        pollenTargetVelocity = speed;
    }

    public void setNectarFlywheelSpeed(double speed) {
        nectarTargetVelocity = speed;
    }

    public void updateFlywheels() {
        pollenFlywheel.setVelocity(pollenTargetVelocity);
        nectarFlywheel.setVelocity(nectarTargetVelocity);
    }

    //Interpolation for flywheels and corresponding hoods. Use data from Desmos, input distance to goals and working values using the flywheel interpolation tuner
    //to find the correct values, the functions below will adjust them automatically when you calculate the distance to the goals.

    //Pollen Interpolation, input math equation from data into the Math.min() section
    public static double pollenFlywheelInterpolated(double distanceGoal) {
        return Math.max(0, Math.min(1300, 0.0115741*Math.pow(distanceGoal,3) - 1.31944*Math.pow(distanceGoal, 2)+50*distanceGoal+606));
    }

    //Nectar Flywheel interpolation, input math equation from data into the Math.min() section
    public static double nectarFlywheelInterpolated(double distanceGoal) {
        return Math.max(0, Math.min(1350, -0.0270062*Math.pow(distanceGoal,3) + 3.44246*Math.pow(distanceGoal, 2)-140.27778*distanceGoal+3099.14286));
    }

    //Pollen Hood interpolation, input math equation from data into the Math.min() section
    public static double pollenHoodInterpolated(double distanceGoal) {
        return Math.max(0.0, Math.min(0.45, -0.01*distanceGoal+0.76));
    }

    //Nectar Hood interpolation, input math equation from data into the Math.min() section
    public static double nectarHoodInterpolated(double distanceGoal) {
        return Math.max(0.0, Math.min(0.85, -0.0000192901*Math.pow(distanceGoal,3) + 0.00213294*Math.pow(distanceGoal, 2)-0.0638889*distanceGoal+1.07143));
    }

    public boolean isDoneShooting() {
        return shotsRemaining == 0 && flywheelState == FlywheelState.IDLE;
    }

    public void setHoodAngles(double pollenAngle, double nectarAngle) {
        hood.setPollenHoodPos(pollenAngle);
        hood.setNectarHoodPos(nectarAngle);
    }

    public void setIntake(Intake intake) {
        this. intake = intake;
    }

    public double getAmperagePollen() {
        return pollenFlywheel.getCurrent(CurrentUnit.AMPS);
    }

    public double getAmperageNectar() {
        return nectarFlywheel.getCurrent(CurrentUnit.AMPS);
    }
}
