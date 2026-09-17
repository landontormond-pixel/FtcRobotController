package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RedGoalPedro extends OpMode {
    DcMotorEx launch;
    DcMotor intake;
    CRServo intakeLeft;
    CRServo intakeRight;
    CRServo Gate1;
    CRServo Gate2;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {

        Drive_StartPos_ShootPos,
        Drive_Shoot_Two,

        Drive_ShootPos_PrePick1,

        Drive_PrePick1_Pick1,
        Drive_Pick1_ShootPos,
        Drive_Shoot_Two_After_Pick1,

        Drive_ShootPos_PrePick2,
        Drive_ShootPos_EndPos,
    }

    PathState pathState;

    private final Pose startPose = new Pose(57.01876675603217, 8.093833780160859, Math.toRadians(90));
    private final Pose shootPose = new Pose(69.0965147453083, 21.02680965147453, Math.toRadians(120));
    private final Pose prepick1Pose = new Pose(47.67828418230564, 35.2064343163539, Math.toRadians(180));
    private final Pose pick1Pose = new Pose(12.128686327077748, 35.85790884718499, Math.toRadians(180));
    private final Pose endPose = new Pose(35.096514745308305, 11.876675603217148, Math.toRadians(120));
    boolean pathStarted = false;

    private PathChain driveStartToShootPos, driveShootToPrePick1 , drivePrePick1ToPick1, drivePick1ToShoot, driveShootToEnd;
    public void buildPaths(){

        driveStartToShootPos = follower.pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToPrePick1 = follower.pathBuilder()
                .addPath((new BezierLine(shootPose, prepick1Pose)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), prepick1Pose.getHeading())
                .build();

        drivePrePick1ToPick1 = follower.pathBuilder()
                .addPath((new BezierLine(prepick1Pose, pick1Pose)))
                .setLinearHeadingInterpolation(prepick1Pose.getHeading(), pick1Pose.getHeading())
                .build();

        drivePick1ToShoot = follower.pathBuilder()
                .addPath((new BezierLine(pick1Pose, shootPose)))
                .setLinearHeadingInterpolation(pick1Pose.getHeading(), shootPose.getHeading())
                .build();


        driveShootToEnd = follower.pathBuilder()
                .addPath((new BezierLine(shootPose, endPose)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();

    }

    public void statePathUpdate() {

        switch(pathState){

            case Drive_StartPos_ShootPos:
            if (!pathStarted) {
                follower.followPath(driveStartToShootPos, true);
                pathStarted = true;
            }
            if (!follower.isBusy()) {
                pathStarted = false;
                setPathState(PathState.Drive_Shoot_Two);
            }
            break;


            case Drive_Shoot_Two:

                // keep correcting position
                follower.update();

                double t = pathTimer.getElapsedTimeSeconds();

                // spin up launcher immediately
                launch.setVelocity(1800);

                // ---- SHOT 1 ----
                // fire between 0.40s and 0.70s
                if (t > 0.40 && t < 0.70) {
                    Gate1.setPower(1);
                    Gate2.setPower(1);
                }


                // ---- SHOT 2 ----
                // fire between 1.20s and 1.50s
                if (t > 1.20 && t < 1.50) {
                    intake.setPower(0.5);
                }

                // stop gates after shot 2
                if (t >= 1.50) {
                    Gate1.setPower(0);
                    Gate2.setPower(0);
                }

                // after 1.50s, move to next path
                if (t > 1.50) {
                    setPathState(PathState.Drive_ShootPos_PrePick1);
                }

                break;

            case Drive_ShootPos_PrePick1:
                if (!pathStarted) {
                    follower.followPath(driveShootToPrePick1, true);
                    pathStarted = true;
                } if (!follower.isBusy()) {
                pathStarted = false;
                setPathState(PathState.Drive_PrePick1_Pick1);
            }
                break;

            case Drive_PrePick1_Pick1:
                    if (!pathStarted) {
                        follower.followPath(drivePrePick1ToPick1, true);
                        pathStarted = true;
                    } if (!follower.isBusy()) {
                        pathStarted = false;
                        setPathState(PathState.Drive_Pick1_ShootPos);
                    }
                    break;
                case Drive_Pick1_ShootPos:
                        if (!pathStarted) {
                            follower.followPath(drivePick1ToShoot, true);
                            pathStarted = true;
                        } if (!follower.isBusy()) {
                            pathStarted = false;
                            setPathState(PathState.Drive_Shoot_Two_After_Pick1);
                        }
                        break;

            case Drive_Shoot_Two_After_Pick1:

                // keep correcting position
                follower.update();

                t = pathTimer.getElapsedTimeSeconds();

                // spin up launcher immediately
                launch.setVelocity(1800);

                // ---- SHOT 1 ----
                // fire between 0.40s and 0.70s
                if (t > 0.40 && t < 0.70) {
                    Gate1.setPower(1);
                    Gate2.setPower(1);
                }


                // ---- SHOT 2 ----
                // fire between 1.20s and 1.50s
                if (t > 1.20 && t < 1.50) {
                    intake.setPower(0.5);
                }

                // stop gates after shot 2
                if (t >= 1.50) {
                    Gate1.setPower(0);
                    Gate2.setPower(0);
                }

                // after 1.50s, move to next path
                if (t > 1.50) {
                    setPathState(PathState.Drive_ShootPos_PrePick2);
                }

                break;
                 case Drive_ShootPos_EndPos:
                 if (!pathStarted) {
                 follower.followPath(driveShootToEnd, true);
                 pathStarted = true;
                 }
                 if (!follower.isBusy()) {
                 pathStarted = false;
                 telemetry.addLine("All Paths Finished");
                 }

                 break;

                default: telemetry.addLine("No State Commanded");

            break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        intake = hardwareMap.get(DcMotor.class, "intake");

        //Servo Setup
        Gate1 = hardwareMap.get(CRServo.class, "Gate1");
        Gate2 = hardwareMap.get(CRServo.class, "Gate2");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");



        //Set Motor and Servo Direction
        Gate1.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        pathState = PathState.Drive_StartPos_ShootPos;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        launch.setVelocity(1600);
        intakeLeft.setPower(1);
        intakeRight.setPower(1);
    }

    public void loop() {

        follower.update();
        statePathUpdate();

        telemetry.addData("Path State" , pathState.toString());
        telemetry.addData("X" , follower.getPose().getX());
        telemetry.addData("Y" , follower.getPose().getY());
        telemetry.addData("Heading" , follower.getPose().getHeading());
        telemetry.addData("Path Time" , pathTimer.getElapsedTimeSeconds());
    }
}
