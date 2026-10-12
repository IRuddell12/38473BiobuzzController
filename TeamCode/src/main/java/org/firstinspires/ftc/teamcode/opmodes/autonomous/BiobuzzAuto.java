package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.parallel;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import static org.firstinspires.ftc.teamcode.mechanisms.commands.Commands.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;


import org.firstinspires.ftc.teamcode.pedro.PoseHolder;

@Autonomous(name = "Auto", group = "Autonomous")
public class BiobuzzAuto extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private LimelightVision limelight;

    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousOptions = false;

    private static final double FIELD_SIZE = 144.0;

    private Pose alliancePose(Pose redPose) {
        if (RobotSettings.allianceColor != RobotSettings.AllianceColor.BLUE) {
            return redPose;
        }

        double blueX = FIELD_SIZE - redPose.x();
        double blueY = FIELD_SIZE - redPose.y();
        double blueHeading = (redPose.heading() + 180.0) % 360.0;

        return p.of(blueX, blueY, blueHeading);
    }

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(55.1914, 8.8086, 90);
    private final Pose flower1Pose = p.of(11.7307, 47.32, 180);
    private final Pose startToFlower1Control = p.of(31.0921, 49.9236, 0);
    private final Pose startToFlower1_Segment1Start = p.of(11.7307, 47.32, 270);
    private final Pose startToFlower1_Segment1End = p.of(11.7307, 47.32, 180);
    private final Pose startToFlower1_Segment2Start = p.of(11.7307, 47.32, 180);
    private final Pose startToFlower1_Segment2End = p.of(11.7307, 47.32, 180);


    private final Pose shootPose1 = p.of(58, 24, 270);
    private final Pose shootPose2 = p.of(58, 118, 110);
    private final Pose flower1ToShoot2Control1 = p.of(34, 48, 0);
    private final Pose flower1ToShoot2Control2 = p.of(18, 110, 0);

    private final Pose shoot2ToFlower2Control = p.of(47, 118, 0);

    private final Pose shootToShoot_Segment1Start = p.of(58, 24, 90);
    private final Pose ShootToShoot_Segment1End = p.of(58, 24, 180);
    private final Pose shootToShoot_Segment2Start = p.of(58, 24, 180);
    private final Pose shootToShoot_Segment2End = p.of(58, 24, 180);
    private final Pose shootToShoot_Segment3Start = p.of(58, 24, 180);
    private final Pose shootToShoot_Segment3End = p.of(58, 24, 270);

    private final Pose flower2Pose = p.of(47.2343, 129.5021, 90);
    private final Pose parkPose = p.of(15.5179, 110.09, 270);

    private final double flowerIntakeTimeout = 2.0;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                shootPreloads(),
                runCycle1(),
                runCycle2(),
                parallel(follow(follower, shoot2ToPark()))
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        PoseHolder.savedPose = null;

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        shooter = new Shooter();
        shooter.init(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);

        limelight = new LimelightVision();
        limelight.init(hardwareMap);

        follower.update();

    }

    @Override
    public void init_loop() {
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean options = gamepad1.options;

        // Options resets the alliance selection.
        if (options && !previousOptions) {
            RobotSettings.allianceColor = null;
        }

        // Select alliance on a new button press.
        if (leftBumper && !previousLeftBumper) {
            RobotSettings.allianceColor =
                    RobotSettings.AllianceColor.BLUE;
        } else if (rightBumper && !previousRightBumper) {
            RobotSettings.allianceColor =
                    RobotSettings.AllianceColor.RED;
        }

        telemetry.addLine("=== WARRIOR DYNAMICS ===");
        telemetry.addLine("Select Alliance");
        telemetry.addLine("Left Bumper: BLUE");
        telemetry.addLine("Right Bumper: RED");
        telemetry.addLine("Options: Reset Selection");

        telemetry.addData(
                "Selected Alliance",
                RobotSettings.allianceColor == null
                        ? "NOT SELECTED"
                        : RobotSettings.allianceColor
        );

        telemetry.addLine(
                RobotSettings.allianceColor == null
                        ? "Select an alliance before starting."
                        : "Alliance selected."
        );

        telemetry.update();

        previousLeftBumper = leftBumper;
        previousRightBumper = rightBumper;
        previousOptions = options;
    }


    @Override
    public void start() {
        if (RobotSettings.allianceColor == null) {
            telemetry.addLine("Select an alliance first.");
            telemetry.update();
            return;
        }

        schedule(autoRoutine());
    }

    public void loop(){
            follower.update();
            shooter.update();
            limelight.update();
            intake.updateIntakeDetection();
            Scheduler.execute();

            PoseHolder.savedPose = follower.pose();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }

    @Override
    public void stop() {
        if (follower != null) {
            PoseHolder.savedPose = follower.pose();
        }
    }


    public Path startToFlower1() {
        return Paths.curve(
                alliancePose(startPose),
                alliancePose(startToFlower1Control),
                alliancePose(flower1Pose)
        ).heading(Interpolator.piecewise()
                .until(0.8, Interpolator.linear(
                        alliancePose(startToFlower1_Segment1Start),
                        alliancePose(startToFlower1_Segment1End)))
                .until(1, Interpolator.linear(
                        alliancePose(startToFlower1_Segment2Start),
                        alliancePose(startToFlower1_Segment2End))));
    }

    public Path flower1ToShoot2() {
        Pose start = alliancePose(flower1Pose);
        Pose end = alliancePose(shootPose2);

        return Paths.curve(
                start,
                alliancePose(flower1ToShoot2Control1),
                alliancePose(flower1ToShoot2Control2),
                end
        ).linear(start, end);
    }

    public Path shoot2ToFlower2() {
        Pose start = alliancePose(shootPose2);
        Pose end = alliancePose(flower2Pose);

        return Paths.curve(
                start,
                alliancePose(shoot2ToFlower2Control),
                end
        ).linear(start, end);
    }

    public Path flower2ToShoot2() {
        Pose start = alliancePose(flower2Pose);
        Pose end = alliancePose(shootPose2);

        return Paths.curve(
                start,
                alliancePose(shoot2ToFlower2Control),
                end
        ).linear(start, end);
    }

    public Path shoot2ToPark() {
        Pose start = alliancePose(shootPose2);
        Pose end = alliancePose(parkPose);

        return Paths.line(start, end).linear(start, end);
    }

    public Path shoot2Toshoot1() {
        Pose start = alliancePose(shootPose2);
        Pose end = alliancePose(shootPose1);

        return Paths.line(start, end).heading(Interpolator.piecewise()
                .until(0.2, Interpolator.linear(
                        alliancePose(shootToShoot_Segment1Start),
                        alliancePose(ShootToShoot_Segment1End)))
                .until(0.8, Interpolator.linear(
                        alliancePose(shootToShoot_Segment2Start),
                        alliancePose(shootToShoot_Segment2End)))
                .until(1, Interpolator.linear(
                        alliancePose(shootToShoot_Segment3Start),
                        alliancePose(shootToShoot_Segment3End))));
    }


    private Command checkThenShoot(int numberOfShots, double timeoutSeconds) {
        boolean[] scorable = {false};
        return sequential(
                checkRedCellScorable(limelight, timeoutSeconds, scorable),

                conditional(() -> scorable[0],
                        shoot(shooter, numberOfShots),
                        Command.build().setDone(() -> true)
                )
        );
    }

    private Command shootPreloads() {
        return sequential(shoot(shooter, 4)
        );
    }

    private Command waitForFullCapacity() {
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(timer::reset)
                .setDone(() ->
                        intake.isFull()
                                || timer.seconds() >= flowerIntakeTimeout
                );
    }

    private Command runCycle1() {
        return sequential(
                parallel(
                        follow(follower, startToFlower1()),
                        runAtProgress(follower, intake, .8, 1, 1)),
                parallel(
                        follow(follower, flower1ToShoot2()),
                        runAtProgress(follower, intake, .2, 0, 0)),
                shoot(shooter, 4)
        );
    }

    private Command runCycle2() {
        return sequential(
                parallel(
                        follow(follower, shoot2ToFlower2()),
                        runIntake(intake, 1),
                        setProbe(intake, 1)),
                waitForFullCapacity(),
                follow(follower, flower2ToShoot2()),
                parallel(
                        shoot(shooter, 4),
                        setProbe(intake,0))
        );
    }

    private Command searchCheckShootTransition() {
        return sequential(

        );
    }
}
