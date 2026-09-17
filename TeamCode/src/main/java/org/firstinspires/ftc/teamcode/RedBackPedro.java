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
public class RedBackPedro extends OpMode {
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
        Drive_Shoot_Two_After_Pick2,
        Drive_Shoot_Two_After_Pick3,
        Drive_ShootPos_PrePick2,
        Drive_PrePick2_Pick2,
        Drive_Pick2_ShootPos,
        Drive_ShootPos_PrePick3,
        Drive_PrePick3_Pick3,
        Drive_Pick3_ShootPos,
        Drive_ShootPos_EndPos,
    }

    PathState pathState;

    private final Pose startPose = new Pose(123.0348525469169, 121.98123324396782, Math.toRadians(44));
    private final Pose shootPose = new Pose(95.06970509383378, 94.80428954423591, Math.toRadians(44));
    private final Pose prepick1Pose = new Pose(83.77747989276139, 83.73190348525466, Math.toRadians(0));
    private final Pose pick1Pose = new Pose(129.2278820375335, 83.58445040214481, Math.toRadians(0));
    private final Pose prepick2Pose = new Pose(95.54959785522787, 59.88471849865951, Math.toRadians(0));
    private final Pose pick2Pose = new Pose(131.2520107238606, 59.82573726541554, Math.toRadians(0));
    private final Pose prepick3Pose = new Pose(95.29222520107238, 35.6461126005362, Math.toRadians(0));
    private final Pose pick3Pose = new Pose(131.1206434316354, 35.747989276139435, Math.toRadians(0));
    private final Pose endPose = new Pose(106.15013404825737, 72.5603217158177, Math.toRadians(0));

    double t = pathTimer.getElapsedTimeSeconds();
    boolean pathStarted = false;

    private PathChain driveStartToShootPos, driveShootToPrePick1 , drivePrePick1ToPick1, drivePick1ToShoot,
            driveShootToPrePick2, drivePrePick2ToPick2, drivePick2ToShoot, driveShootToPrePick3,
            drivePrePick3ToPick3, drivePick3ToShoot, driveShootToEnd;
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

        driveShootToPrePick2 = follower.pathBuilder()
                .addPath((new BezierLine(shootPose, prepick2Pose)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), prepick2Pose.getHeading())
                .build();

        drivePrePick2ToPick2 = follower.pathBuilder()
                .addPath((new BezierLine(prepick2Pose, pick2Pose)))
                .setLinearHeadingInterpolation(prepick2Pose.getHeading(), pick2Pose.getHeading())
                .build();

        drivePick2ToShoot = follower.pathBuilder()
                .addPath((new BezierLine(pick2Pose, shootPose)))
                .setLinearHeadingInterpolation(pick2Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToPrePick3 = follower.pathBuilder()
                .addPath((new BezierLine(shootPose, prepick3Pose)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), prepick3Pose.getHeading())
                .build();

        drivePrePick3ToPick3 = follower.pathBuilder()
                .addPath((new BezierLine(prepick3Pose, pick3Pose)))
                .setLinearHeadingInterpolation(prepick3Pose.getHeading(), pick3Pose.getHeading())
                .build();

        drivePick3ToShoot = follower.pathBuilder()
                .addPath((new BezierLine(pick3Pose, shootPose)))
                .setLinearHeadingInterpolation(pick3Pose.getHeading(), shootPose.getHeading())
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

            case Drive_ShootPos_PrePick1:
                if (!pathStarted) {
                    follower.followPath(driveShootToPrePick1, true);
                    pathStarted = true;
                } if (!follower.isBusy()) {
                    pathStarted = false;
                    setPathState(PathState.Drive_PrePick1_Pick1);
                }
                break;

            case Drive_Shoot_Two:

                // keep correcting position
                follower.update();

                t = pathTimer.getElapsedTimeSeconds();

                // spin up launcher immediately
                launch.setVelocity(1600);

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
                launch.setVelocity(1600);

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

                case Drive_ShootPos_PrePick2:
                            if (!pathStarted) {
                                follower.followPath(driveShootToPrePick2, true);
                                pathStarted = true;
                            } if (!follower.isBusy()) {
                                pathStarted = false;
                                setPathState(PathState.Drive_PrePick2_Pick2);
                            }
                            break;
                case Drive_PrePick2_Pick2:
                                if (!pathStarted) {
                                    follower.followPath(drivePrePick2ToPick2, true);
                                    pathStarted = true;
                                } if (!follower.isBusy()) {
                                    pathStarted = false;
                                    setPathState(PathState.Drive_Pick2_ShootPos);
                                }
                                break;
                case Drive_Pick2_ShootPos:
                                    if (!pathStarted) {
                                        follower.followPath(drivePick2ToShoot, true);
                                        pathStarted = true;
                                    } if (!follower.isBusy()) {
                                        pathStarted = false;
                                        setPathState(PathState.Drive_ShootPos_PrePick3);
                                    } break;

            case Drive_Shoot_Two_After_Pick2:

                // keep correcting position
                follower.update();

                t = pathTimer.getElapsedTimeSeconds();

                // spin up launcher immediately
                launch.setVelocity(1600);

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
                    setPathState(PathState.Drive_ShootPos_PrePick3);
                }

                break;


                case Drive_ShootPos_PrePick3:
                                        if (!pathStarted) {
                                            follower.followPath(driveShootToPrePick3, true);
                                            pathStarted = true;
                                        } if (!follower.isBusy()) {
                                            pathStarted = false;
                                            setPathState(PathState.Drive_PrePick3_Pick3);
                                        }
                                        break;
                case Drive_PrePick3_Pick3:
                                                if (!pathStarted) {
                                                follower.followPath(drivePrePick3ToPick3, true);
                                                pathStarted = true;
                                            } if (!follower.isBusy()) {
                                                pathStarted = false;
                                                setPathState(PathState.Drive_Pick3_ShootPos);
                                            } break;
                                            case Drive_Pick3_ShootPos:
                                                if (!pathStarted) {
                                                    follower.followPath(drivePick3ToShoot, true);
                                                    pathStarted = true;
                                                } if (!follower.isBusy()) {
                                                    pathStarted = false; setPathState(PathState.Drive_ShootPos_EndPos);
                                                }
                                                break;

            case Drive_Shoot_Two_After_Pick3:

                // keep correcting position
                follower.update();

                t = pathTimer.getElapsedTimeSeconds();

                // spin up launcher immediately
                launch.setVelocity(1600);

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
