package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.*;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class RedFarPark extends OpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(46.3993, 8.4874, 90);
    private final Pose pickupPose = poseFactory.of(9.3755, 22.338, -90);
    private final Pose collectPose = poseFactory.of(8.0308, 10.1631, -90);
    private final Pose launchPose = poseFactory.of(56.8674, 31.0758, 90);
    private final Pose parkPose = poseFactory.of(8.618, 98.1767, 0);

    private Path StartToPickup() {
        return line(startPose, pickupPose).linear(startPose, pickupPose);
    }
    private Path PickupToCollect() {
        return line(pickupPose, collectPose).linear(pickupPose, collectPose);
    }
    private Path CollectToLaunch() {
        return line(collectPose, launchPose).linear(collectPose, launchPose);
    }
    private Path LaunchToPark() {
        return line(launchPose, parkPose).linear(launchPose, parkPose);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, StartToPickup()),

                follow(follower, PickupToCollect()),

                follow(follower, CollectToLaunch()),

                waitMs(3000),

                follow(follower, LaunchToPark())
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
    }
}