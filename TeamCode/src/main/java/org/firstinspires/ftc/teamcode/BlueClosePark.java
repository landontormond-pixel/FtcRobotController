package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red Far Park", group = "Autonomous")
public class BlueClosePark extends LinearOpMode {

    private DcMotorEx Cannon;
    private DcMotor Intake;
    private CRServo Launcher;


    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(118.75, 7.75, 90);
    private final Pose startLaunch1 = poseFactory.of(79.75, 10.75, 95);
    private final Pose startLaunch1Control1 = poseFactory.of(94.75, 20.75, 0);
    private final Pose launch1ParkStart = poseFactory.of(79.75, 10.75, 100);
    private final Pose launch1Park = poseFactory.of(127.75, 23.75, 180);
    private final Pose launch1ParkControl1 = poseFactory.of(94.75, 20.75, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startLaunch1()),

                launchBalls(1.0, 3.0),

                follow(follower, launch1Park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
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
        return Paths.curve(start, startLaunch1Control1, startLaunch1).linear(start, startLaunch1);
    }

    public Path launch1Park() {
        return Paths.curve(launch1ParkStart, launch1ParkControl1, launch1Park).linear(launch1ParkStart, launch1Park);
    }
}
