package org.firstinspires.ftc.teamcode.mechanisms.commands;

import static com.pedropathing.ivy.commands.Commands.instant;

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
                .setStart(() -> shooter.setPollenFlywheelSpeed(velocity))
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
        return instant(() -> intake.setProbe(position));
    }

    public static Command setIntakeLift(Intake intake, double position) {
        return instant(() -> intake.setIntakeLift(position));
    }
//------------------SPECIAL AUTO COMMANDS-----------------------------

    public static Command setProbeAtProgress(
            Follower follower,
            Intake intake,
            double progressThreshold,
            int probeValue
    ) {
        boolean[] pathStarted = {false};
        boolean[] triggered = {false};

        return Command.build()
                .setStart(() -> {
                    pathStarted[0] = false;
                    triggered[0] = false;
                })
                .setExecute(() -> {

                    // Wait until the new path actually starts
                    if (follower.isBusy()) {
                        pathStarted[0] = true;
                    }

                    // Trigger once the desired progress is reached
                    if (pathStarted[0]
                            && !triggered[0]
                            && follower.parametricCompletion() >= progressThreshold) {

                        intake.setProbe(probeValue);
                        triggered[0] = true;
                    }
                })
                .setDone(() -> pathStarted[0] && !follower.isBusy())
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
