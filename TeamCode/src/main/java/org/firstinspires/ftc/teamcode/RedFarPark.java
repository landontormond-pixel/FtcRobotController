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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red Far Park", group = "Autonomous")
public class RedFarPark extends LinearOpMode {

    private DcMotorEx Cannon;
    private DcMotor Intake;
    private CRServo Launcher;


    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(46.8441, 8.6243, 90);
    private final Pose startLaunch1 = poseFactory.of(58.1607, 8.8438, 90);
    private final Pose launch1PickupStart = poseFactory.of(58.1607, 8.8438, 90);
    private final Pose launch1Pickup = poseFactory.of(10.4203, 31.735, -90);
    private final Pose pickupCollectStart = poseFactory.of(10.4203, 31.735, -90);
    private final Pose pickupCollect = poseFactory.of(9.5036, 11.0038, -90);
    private final Pose collectLaunch2Start = poseFactory.of(9.5036, 11.0038, -90);
    private final Pose collectLaunch2 = poseFactory.of(58.6802, 129.01, -90);
    private final Pose collectLaunch2Control1 = poseFactory.of(18.5805, 31.9522, 0);
    private final Pose collectLaunch2Control2 = poseFactory.of(50.5146, 55.954, 0);
    private final Pose collectLaunch2Control3 = poseFactory.of(0, 64.7634, 0);
    private final Pose launch2Pickup2Start = poseFactory.of(58.6802, 129.01, -90);
    private final Pose launch2Pickup2 = poseFactory.of(45.5767, 109.1704, 90);
    private final Pose pickup2Collect2Start = poseFactory.of(45.5767, 109.1704, 90);
    private final Pose pickup2Collect2 = poseFactory.of(46.3025, 126.7928, 90);
    private final Pose collect2Launch3Start = poseFactory.of(46.3025, 126.7928, 90);
    private final Pose collect2Launch3 = poseFactory.of(58.68, 129.01, -90);
    private final Pose collect2Launch3Control1 = poseFactory.of(53.4578, 105.8963, 0);
    private final Pose launch3ParkStart = poseFactory.of(58.68, 129.01, -90);
    private final Pose launch3Park = poseFactory.of(13.7448, 91.4211, 0);
    private final Pose launch3ParkControl1 = poseFactory.of(15.912, 80.019, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startLaunch1()),

                launchBalls(1.0, 3.0),

                follow(follower, launch1Pickup()),

                follow(follower, pickupCollect()),

                follow(follower, collectLaunch2()),

                launchBalls(1.0, 3.0),

                follow(follower, launch2Pickup2()),

                follow(follower, pickup2Collect2()),

                follow(follower, collect2Launch3()),

                launchBalls(1.0, 3.0),

                follow(follower, launch3Park())
        );
    }



    @Override
    public void runOpMode() {

        Cannon = hardwareMap.get(DcMotorEx.class, "Cannon");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Launcher = hardwareMap.get(CRServo.class, "Launcher");

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        Cannon.setVelocity(1300);
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

        // Stop motors when autonomous ends
        Cannon.setVelocity(0);
        Launcher.setPower(0);
        Intake.setPower(0);
    }


    public Command launchBalls(double power, double seconds) {
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(() -> {
                    timer.reset();
                })
                .setExecute(() -> {
                    Launcher.setPower(power);
                })
                .setDone(() -> {
                    return timer.seconds() >= seconds;
                })
                .setEnd(endCondition -> {
                    Launcher.setPower(0);
                })
                .requiring(Launcher);
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
        return Paths.curve(collectLaunch2Start, collectLaunch2Control1, collectLaunch2Control2, collectLaunch2Control3, collectLaunch2).linear(collectLaunch2Start, collectLaunch2);
    }

    public Path launch2Pickup2() {
        return Paths.line(launch2Pickup2Start, launch2Pickup2).linear(launch2Pickup2Start, launch2Pickup2);
    }

    public Path pickup2Collect2() {
        return Paths.line(pickup2Collect2Start, pickup2Collect2).linear(pickup2Collect2Start, pickup2Collect2);
    }

    public Path collect2Launch3() {
        return Paths.curve(collect2Launch3Start, collect2Launch3Control1, collect2Launch3).linear(collect2Launch3Start, collect2Launch3);
    }

    public Path launch3Park() {
        return Paths.curve(launch3ParkStart, launch3ParkControl1, launch3Park).linear(launch3ParkStart, launch3Park);
    }
}
