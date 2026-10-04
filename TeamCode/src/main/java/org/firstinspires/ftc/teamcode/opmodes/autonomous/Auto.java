package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous
public class Auto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    //p.of(x, y, heading) this is the new coordinate system
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose parkPose = p.of(48, 48, 90);
    private final Pose scorePose = p.of(36, 60, 45);


    private  Path startToScore() {
        return line(startPose, parkPose).linear(startPose, parkPose);
    }

    private Path park() {
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }

    private Command autoRoutine() {
        return  sequential(
                follow(follower, startToScore()),
                //Score
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}