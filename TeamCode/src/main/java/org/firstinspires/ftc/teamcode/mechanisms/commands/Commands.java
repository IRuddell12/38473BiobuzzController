package org.firstinspires.ftc.teamcode.mechanisms.commands;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;


//-----------------------SHOOTER COMMANDS-----------------------------

public class Commands {

    // Fires a specified number of shots
    public static Command shoot(Shooter shooter, int numberOfShots) {

        return Command.build()
                .setStart(() -> shooter.fireShots(numberOfShots))
                .setDone(shooter::isDoneShooting)
                .requiring(shooter);
    }

    //Cancels the current shooting sequence
    public static Command cancelShooting(Shooter shooter) {

        return Command.build()
                .setStart(shooter::cancelShooting)
                .setDone(() -> true)
                .requiring(shooter);
    }

    //Sets the flywheel velocity
    public static Command setFlywheelSpeed(
            Shooter shooter,
            double velocity) {

        return Command.build()
                .setStart(() -> shooter.setFlywheelSpeed(velocity))
                .setDone(() -> true)
                .requiring(shooter);
    }

//-----------------------INTAKE COMMANDS-----------------------------

    public static Command runIntake(Intake intake, double power) {
        return Command.build()
                .setStart(() -> intake.runIntake(power))
                .setExecute(() -> intake.runIntake(power))
                .requiring(intake);
    }

//-----------------------PROBE COMMANDS-------------------------------

    public static Command setProbe(Intake intake, int position) {
        return Command.build()
                .setStart(() -> intake.setProbe(position))
                .setExecute(() -> intake.setProbe(position))
                .requiring(intake);
    }

//------------------SPECIAL AUTO COMMANDS-----------------------------

    public static Command runAtProgress(
            Follower follower,
            Intake intake,
            double progressThreshold,
            double intakePower,
            int probeValue
    ) {
        return Command.build()
                .setExecute(() -> {
                    if (follower.completion() >= progressThreshold) {
                        intake.setProbe(probeValue);
                        intake.runIntake(intakePower);
                    }
                })
                .setDone(() -> follower.completion() >= progressThreshold)
                .requiring(intake);
    }

//--------------------------LIMELIGHT COMMANDS----------------------------



    public static Command checkRedCellScorable(LimelightVision limelight, double timeoutSeconds, boolean[] scorableResult) {

        ElapsedTime timer = new ElapsedTime();
        boolean[] finished = {false};

        return Command.build()
                .setStart(() -> {
                    scorableResult[0] = false;
                    finished[0] = false;
                    timer.reset();
                    limelight.start();
                })
                .setExecute(() -> {
                    if (limelight.seesRedTag() && limelight.isScorable()) {
                        scorableResult[0] = true;
                        finished[0] = true;
                    }
                })
                .setDone(() -> finished[0] || timer.seconds() >= timeoutSeconds)
                .setEnd(endCondition -> limelight.stop());
    }
}
