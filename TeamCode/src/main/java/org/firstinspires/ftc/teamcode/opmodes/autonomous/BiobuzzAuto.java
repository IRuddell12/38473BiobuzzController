package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.pedropathing.api.Paths;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;


import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.parallel;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.paths.interpolator.Interpolator;


import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.Probe;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.commands.MechanismCommands;
import org.firstinspires.ftc.teamcode.pedro.Constants;


import org.firstinspires.ftc.teamcode.pedro.PoseHolder;

@Autonomous(name = "Auto", group = "Autonomous")
public class BiobuzzAuto extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Probe probe;
    private Intake intake;
    private LimelightVision limelight;

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(55.1914, 8.8086, 90);
    private final Pose flower1Pose = p.of(11.7307, 47.32, 180);
    private final Pose startToFlower1Control = p.of(31.0921, 49.9236, 0);
    private final Pose startToFlower1_Segment1Start = p.of(11.7307, 47.32, 270);
    private final Pose startToFlower1_Segment1End = p.of(11.7307, 47.32, 180);
    private final Pose startToFlower1_Segment2Start = p.of(11.7307, 47.32, 180);
    private final Pose startToFlower1_Segment2End = p.of(11.7307, 47.32, 180);
    
    private final Pose shootPose2 = p.of(47.4029, 124.0457, 110);
    private final Pose flower1ToShoot2Control1 = p.of(31.4, 49.1807, 0);
    private final Pose flower1ToShoot2Control2 = p.of(20.4414, 102.9264, 0);

    private final Pose flower2Pose = p.of(47.2343, 129.5021, 90);
    private final Pose parkPose = p.of(15.5179, 110.09, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                MechanismCommands.shootWhenRedCellScorable(shooter, limelight, 4, 3.0),
                parallel(follow(follower, startToFlower1()),
                        MechanismCommands.runAtProgress(follower, intake, probe, 1, .8, 1)),
                parallel(follow(follower, flower1ToShoot2()),
                        MechanismCommands.runAtProgress(follower, intake, probe, 0, .2, 0)),
                parallel(MechanismCommands.shootWhenRedCellScorable(shooter, limelight, 4, 3.0)),
                        MechanismCommands.runIntake(intake, .5),
                parallel(follow(follower, shoot2ToFlower2()), MechanismCommands.runIntake(intake, 1), MechanismCommands.setProbe(probe, 1)),
                follow(follower, flower2ToShoot2()),
                parallel(MechanismCommands.shootWhenRedCellScorable(shooter, limelight, 4, 3.0),
                        MechanismCommands.runIntake(intake, .5), MechanismCommands.setProbe(probe,0)),
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
    public void start() {
        schedule(autoRoutine());
    }

    public void loop(){
            follower.update();
            shooter.update();
            limelight.update();
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
        return Paths.curve(startPose, startToFlower1Control, flower1Pose).heading(Interpolator.piecewise()
                .until(0.8, Interpolator.linear(startToFlower1_Segment1Start, startToFlower1_Segment1End))
                .until(1, Interpolator.linear(startToFlower1_Segment2Start, startToFlower1_Segment2End)));
    }

    public Path flower1ToShoot2() {
        return Paths.curve(flower1Pose, flower1ToShoot2Control1, flower1ToShoot2Control2, shootPose2)
                .linear(flower1Pose, shootPose2);
    }

    public Path shoot2ToFlower2() {
        return Paths.line(shootPose2, flower2Pose)
                .linear(shootPose2, flower2Pose);
    }

    public Path flower2ToShoot2() {
        return Paths.line(flower2Pose, shootPose2)
                .linear(flower2Pose, shootPose2);
    }

    public Path shoot2ToPark() {
        return Paths.line(shootPose2, parkPose)
                .linear(shootPose2, parkPose);
    }
}
