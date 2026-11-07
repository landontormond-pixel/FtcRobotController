package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red Far Park", group = "Autonomous")
public class RedFarPark extends LinearOpMode {

    private DcMotorEx Cannon;
    private CRServo Launcher;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(46.8441, 8.6243, 90);
    private final Pose startLaunch1 = poseFactory.of(58.1607, 8.8438, 90);
    private final Pose launch1PickupStart = poseFactory.of(58.1607, 8.8438, 90);
    private final Pose launch1Pickup = poseFactory.of(10.4203, 31.735, -90);
    private final Pose pickupCollectStart = poseFactory.of(10.4203, 31.735, -90);
    private final Pose pickupCollect = poseFactory.of(9.2955, 9.027, -90);
    private final Pose collectLaunch2Start = poseFactory.of(9.2955, 9.027, -90);
    private final Pose collectLaunch2 = poseFactory.of(58.8883, 132.9646, -90);
    private final Pose collectLaunch2Control1 = poseFactory.of(9.7367, 103.9507, 0);
    private final Pose collectLaunch2Control2 = poseFactory.of(64.7686, 108.6003, 0);
    private final Pose launch2Pickup2Start = poseFactory.of(58.8883, 132.9646, -90);
    private final Pose launch2Pickup2 = poseFactory.of(45.5767, 109.1704, 90);
    private final Pose pickup2Collect2Start = poseFactory.of(45.5767, 109.1704, 90);
    private final Pose pickup2Collect2 = poseFactory.of(46.7187, 131.4748, 90);
    private final Pose point7Start = poseFactory.of(46.7187, 131.4748, 90);
    private final Pose point7 = poseFactory.of(58.89, 132.96, -90);
    private final Pose point7Control1 = poseFactory.of(53.4578, 105.8963, 0);
    private final Pose point8Start = poseFactory.of(58.89, 132.96, -90);
    private final Pose point8 = poseFactory.of(8.0224, 98.4961, 0);
    private final Pose point8Control1 = poseFactory.of(15.2877, 79.3947, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startLaunch1()),

                waitMs(3000),

                follow(follower, launch1Pickup()),

                follow(follower, pickupCollect()),

                waitMs(500),

                follow(follower, collectLaunch2()),

                waitMs(3000),

                follow(follower, launch2Pickup2()),

                follow(follower, pickup2Collect2()),

                waitMs(500),

                follow(follower, path7()),

                follow(follower, path8())
        );
    }

    private long startTime;
    public Command LaunchBall(DcMotor motor, double power, long milliseconds) {
        return Command.build()
                .setStart(() -> {
                    motor.setPower(power);
                    startTime = System.currentTimeMillis();
                })
                .setDone(() ->
                        System.currentTimeMillis() - startTime >= milliseconds
                )
                .setEnd(endCondition -> motor.setPower(0))
                .requiring(motor);
    }

    @Override
    public void runOpMode() {

        Cannon = hardwareMap.get(DcMotorEx.class, "Cannon");
        Launcher = hardwareMap.get(CRServo.class, "Launcher");

        Cannon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        Cannon.setVelocity(2500);
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path startLaunch1() {
        return Paths.line(start, startLaunch1).linear(start, startLaunch1);
    }

    public Path launch1Pickup() {
        return Paths.line(launch1PickupStart, launch1Pickup).linear(launch1PickupStart, launch1Pickup);
    }

    public Path pickupCollect() {
        return Paths.line(pickupCollectStart, pickupCollect).linear(pickupCollectStart, pickupCollect);
    }

    public Path collectLaunch2() {
        return Paths.curve(collectLaunch2Start, collectLaunch2Control1, collectLaunch2Control2, collectLaunch2).linear(collectLaunch2Start, collectLaunch2);
    }

    public Path launch2Pickup2() {
        return Paths.line(launch2Pickup2Start, launch2Pickup2).linear(launch2Pickup2Start, launch2Pickup2);
    }

    public Path pickup2Collect2() {
        return Paths.line(pickup2Collect2Start, pickup2Collect2).linear(pickup2Collect2Start, pickup2Collect2);
    }

    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.curve(point8Start, point8Control1, point8).linear(point8Start, point8);
    }
}