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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue Far Park For Misses", group = "Autonomous")
public class RedFarParkForMisses extends LinearOpMode {

    private DcMotorEx Cannon;
    private DcMotor Intake;
    private CRServo Launcher;
    private Servo Flipper;
    private CRServo RIntake;
    private CRServo LIntake;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(46.8441, 8.6243, -90);
    private final Pose startLaunch1 = poseFactory.of(58.15, 8.84, -90);
    private final Pose launch1PickupStart = poseFactory.of(58.15, 8.84, -90);
    private final Pose launch1Pickup = poseFactory.of(10.4203, 31.74, -90);
    private final Pose pickupCollectStart = poseFactory.of(10.4203, 31.74, -90);
    private final Pose pickupCollect = poseFactory.of(9.5, 9.5, -90);
    private final Pose collectLaunch2Start = poseFactory.of(9.5, 9.5, -90);
    private final Pose collectLaunch2 = poseFactory.of(58.1536, 8.8703, -90);
    private final Pose launch3ParkStart = poseFactory.of(58.1536, 8.8703, -90);
    private final Pose launch3Park = poseFactory.of(11.7931, 88.6008, 0);
    private final Pose launch3ParkControl1 = poseFactory.of(17.5462, 43.8101, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startLaunch1()),
                follow(follower, launch1Pickup()),
                follow(follower, pickupCollect()),
                follow(follower, collectLaunch2()),
                follow(follower, launch3Park())
        );
    }

    @Override
    public void runOpMode() {


        Cannon = hardwareMap.get(DcMotorEx.class, "Cannon");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Launcher = hardwareMap.get(CRServo.class, "Launcher");
        Flipper = hardwareMap.get(Servo.class, "Flipper");
        LIntake = hardwareMap.get(CRServo.class, "LIntake");
        RIntake = hardwareMap.get(CRServo.class, "RIntake");

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        Flipper.setPosition(0.0);
        Cannon.setVelocity(1300);
        Intake.setPower(1);
        LIntake.setPower(1);
        RIntake.setPower(1);
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
        return Paths.line(start, startLaunch1).linear(start, startLaunch1);
    }

    public Path launch1Pickup() {
        return Paths.line(launch1PickupStart, launch1Pickup).linear(launch1PickupStart, launch1Pickup);
    }

    public Path pickupCollect() {
        return Paths.line(pickupCollectStart, pickupCollect).linear(pickupCollectStart, pickupCollect);
    }

    public Path collectLaunch2() {
        return Paths.line(collectLaunch2Start, collectLaunch2).linear(collectLaunch2Start, collectLaunch2);
    }

    public Path launch3Park() {
        return Paths.curve(launch3ParkStart, launch3ParkControl1, launch3Park).linear(launch3ParkStart, launch3Park);
    }
}
