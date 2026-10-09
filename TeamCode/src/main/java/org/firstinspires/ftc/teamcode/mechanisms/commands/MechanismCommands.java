package org.firstinspires.ftc.teamcode.mechanisms.commands;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Probe;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;


//-----------------------SHOOTER COMMANDS-----------------------------

public class MechanismCommands {

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

    public static Command setProbe(Probe probe, int position) {
        return Command.build()
                .setStart(() -> probe.setProbe(position))
                .setExecute(() -> probe.setProbe(position))
                .requiring(probe);
    }

//------------------SPECIAL AUTO COMMANDS-----------------------------

    public static Command runAtProgress(
            Follower follower,
            Intake intake,
            Probe probe,
            double intakePower,
            double progressThreshold,
            int probeValue
    ) {
        return Command.build()
                .setExecute(() -> {
                    if (follower.completion() >= progressThreshold) {
                        probe.setProbe(probeValue);
                        intake.runIntake(intakePower);
                    }
                })
                .setDone(() -> follower.completion() >= progressThreshold)
                .requiring(intake);
    }

//--------------------------LIMELIGHT COMMANDS----------------------------


    public static Command shootWhenRedCellScorable(
            Shooter shooter,
            LimelightVision vision,
            int numberOfShots,
            double timeoutSeconds) {

        ElapsedTime timer = new ElapsedTime();
        boolean[] fired = {false};

        return Command.build()
                .setStart(() -> {
                    fired[0] = false;
                    timer.reset();
                })
                .setExecute(() -> {
                    if (!fired[0]
                            && vision.seesRedTag()
                            && vision.isScorable()
                            && !shooter.isBusy()) {

                        shooter.fireShots(numberOfShots);
                        fired[0] = true;
                    }
                })
                .setDone(() ->
                        (fired[0] && shooter.isDoneShooting())
                                || timer.seconds() >= timeoutSeconds)
                .requiring(shooter);
    }
}
