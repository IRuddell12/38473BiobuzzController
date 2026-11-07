package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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
    //private LimelightVision limelight;

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(57.58, 5.80, 90);
    private final Pose flower1Pose = p.of(14.42, 43, 0);
    private final Pose startToFlower1Control = p.of(30, 50, 0);
    private final Pose startToFlower1_Segment1Start = p.of(14.42, 43, 90);
    private final Pose startToFlower1_Segment1End = p.of(14.42, 43, 0);
    private final Pose startToFlower1_Segment2Start = p.of(14.42, 43, 0);
    private final Pose startToFlower1_Segment2End = p.of(14.42, 43, 0);


    //private final Pose shootPose1 = p.of(58, 24, 90);
    private final Pose shootPose2 = p.of(58, 120, 270);
    private final Pose flower1ToShoot2Control1 = p.of(34, 48, 0);
    private final Pose flower1ToShoot2Control2 = p.of(18, 110, 0);

    private final Pose shoot2ToFlower2Control = p.of(47, 118, 0);

    /*private final Pose shootToShoot_Segment1Start = p.of(58, 24, 90);
    private final Pose ShootToShoot_Segment1End = p.of(58, 24, 180);
    private final Pose shootToShoot_Segment2Start = p.of(58, 24, 180);
    private final Pose shootToShoot_Segment2End = p.of(58, 24, 180);
    private final Pose shootToShoot_Segment3Start = p.of(58, 24, 180);
    private final Pose shootToShoot_Segment3End = p.of(58, 24, 270);
     */

    private final Pose flower2Pose = p.of(55, 120, 270);
    private final Pose parkPose = p.of(19.11, 116.60, 90);

    private final double flowerIntakeTimeout = 2500; //This is MS, 1000 for 1 second

    private double pollenHoodAngle = 0.35;
    private double nectarHoodAngle = 0.75;

    double intakeLiftUp = 0;
    double intakeLiftDown = 0.52;
    int probeUp = 0;
    int probeDown = 1;

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

        intake = new Intake();
        intake.init(hardwareMap);

        shooter = new Shooter();
        shooter.setIntake(intake);
        shooter.init(hardwareMap);
        shooter.setHoodAngles(pollenHoodAngle, nectarHoodAngle);

//        limelight = new LimelightVision();
//        limelight.init(hardwareMap);

        intake.setIntakeLift(intakeLiftUp);
        intake.setProbe(probeUp);


        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    public void loop(){
            follower.update();
            shooter.update();
            //limelight.update();
            //intake.updateIntakeDetection();
            Scheduler.execute();

        telemetry.addData("Completion", follower.completion());
        telemetry.addData("Parametric Progress",
                follower.parametricCompletion());

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
                 startPose,
                 startToFlower1Control,
                 flower1Pose
        ).heading(Interpolator.piecewise()
                .until(0.8, Interpolator.linear(
                         startToFlower1_Segment1Start,
                         startToFlower1_Segment1End))
                .until(1, Interpolator.linear(
                         startToFlower1_Segment2Start,
                         startToFlower1_Segment2End)));
    }

    public Path flower1ToShoot2() {
        Pose start = flower1Pose;
        Pose end = shootPose2;

        return Paths.curve(
                start,
                flower1ToShoot2Control1,
                flower1ToShoot2Control2,
                end
        ).linear(start, end);
    }

    public Path shoot2ToFlower2() {
        Pose start = shootPose2;
        Pose end = flower2Pose;

        return Paths.curve(
                start,
                shoot2ToFlower2Control,
                end
        ).linear(start, end);
    }

    public Path flower2ToShoot2() {
        Pose start = flower2Pose;
        Pose end = shootPose2;

        return Paths.curve(
                start,
                shoot2ToFlower2Control,
                end
        ).linear(start, end);
    }

    public Path shoot2ToPark() {
        Pose start = shootPose2;
        Pose end = parkPose;

        return Paths.line(start, end).linear(start, end);
    }

//    public Path shoot2Toshoot1() {
//        Pose start = shootPose2;
//        Pose end = shootPose1;
//
//        return Paths.line(start, end).heading(Interpolator.piecewise()
//                .until(0.2, Interpolator.linear(
//                        (shootToShoot_Segment1Start),
//                        (ShootToShoot_Segment1End)))
//                .until(0.8, Interpolator.linear(
//                        (shootToShoot_Segment2Start),
//                        (shootToShoot_Segment2End)))
//                .until(1, Interpolator.linear(
//                        (shootToShoot_Segment3Start),
//                        (shootToShoot_Segment3End))));
//    }




    private Command shootPreloads() {
        return sequential(
                setHoodAngles(nectarHoodAngle, 0.5),
                shoot(shooter, 4));
    }

    private Command setHoodAngles(double nectarAngle, double pollenAngle) {
        return instant(() -> shooter.setHoodAngles(pollenAngle, nectarAngle));
    }

    private Command runCycle1() {
        return sequential(
                parallel(
                        setProbe(intake, probeDown),
                        follow(follower, startToFlower1()),
                        setIntakeLift(intake, intakeLiftDown)
                ),

                // Wait at flower 1
                waitMs(flowerIntakeTimeout),

                parallel(
                        setProbe(intake, probeUp),
                        follow(follower, flower1ToShoot2())
                ),

                shoot(shooter, 4)
        );
    }

    private Command runCycle2() {
        return sequential(
                parallel(
                        setProbe(intake, probeDown),
                        follow(follower, shoot2ToFlower2())
                ),

                // Wait at flower 2
                waitMs(4000),

                follow(follower, flower2ToShoot2()),
                setHoodAngles(nectarHoodAngle, 0.4),

                parallel(
                        shoot(shooter, 4),
                        setProbe(intake, probeDown)
                )
        );
    }
    //    private Command waitForFullCapacity() {
//        ElapsedTime timer = new ElapsedTime();
//
//        return Command.build()
//                .setStart(timer::reset)
//                .setDone(() ->
//                        intake.isFull()
//                                || timer.seconds() >= flowerIntakeTimeout
//                );
//    }

    //    private Command checkThenShoot(int numberOfShots, double timeoutSeconds) {
//        boolean[] scorable = {false};
//        return sequential(
//                checkRedCellScorable(limelight, timeoutSeconds, scorable),
//
//                conditional(() -> scorable[0],
//                        shoot(shooter, numberOfShots),
//                        Command.build().setDone(() -> true)
//                )
//        );
//    }
}
