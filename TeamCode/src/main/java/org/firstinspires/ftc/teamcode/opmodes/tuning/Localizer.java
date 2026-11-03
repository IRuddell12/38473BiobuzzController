package org.firstinspires.ftc.teamcode.opmodes.tuning;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp (name = "Localizer", group = "Localization")
public class Localizer extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(8.85, 8.82, 270);


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();

        Pose robotPose = follower.pose();

        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double robotHeading = robotPose.heading();

        telemetry.addData("X: ", robotX);
        telemetry.addData("Y: ", robotY);
        telemetry.addData("Heading", Math.toDegrees(robotHeading));

        telemetry.update();
    }
}