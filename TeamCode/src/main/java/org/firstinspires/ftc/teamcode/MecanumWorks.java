package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@TeleOp(name = "Mecanum (Works)")
public class MecanumWorks extends OpMode{
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotorEx launch;
    DcMotor intake;
    CRServo Gate1;
    CRServo Gate2;
    CRServo intakeLeft;
    CRServo intakeRight;

    @Override
    public void init () {

        //Variables
        //double launchSpeed = 0.6;

        //Motor Setup
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        intake = hardwareMap.get(DcMotor.class, "intake");

        //Servo Setup
        Gate1 = hardwareMap.get(CRServo.class, "Gate1");
        Gate2 = hardwareMap.get(CRServo.class, "Gate2");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        //Set Motor and Servo Direction
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Gate1.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);

    }

    @Override
    public void start() {
        launch.setPower(0.6);
    }

    double launchSpeed;
    @Override
    public void loop() {
        if(gamepad1.b) {
            intake.setPower(0.5);
        } else {
            intake.setPower(0);
        }
        if(gamepad1.a) {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        } else {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        }

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if(gamepad1.y) {
            forward = forward / 2;
            strafe = strafe / 2;
            turn = turn / 2;
        } else {
            forward = - gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
        }

        frontLeft.setPower (forward - strafe + turn);
        frontRight.setPower (forward + strafe + turn);
        backLeft.setPower (forward + strafe - turn);
        backRight.setPower (forward - strafe - turn);

        if(gamepad1.right_bumper) {
            Gate1.setPower(1);
            Gate2.setPower(1);
        } else {
            Gate1.setPower(0);
            Gate2.setPower(0);
        }

        //Launch Speed Change
        if (gamepad1.dpad_up) {

            launchSpeed = launchSpeed + 0.1;

        } else if(gamepad1.dpad_down) {

            launchSpeed = launchSpeed - 0.1;

        } else if(gamepad1.left_bumper) {

            launchSpeed = 0.75;

        } else if(gamepad1.dpad_left) {

            launchSpeed = launchSpeed + 0.05;
        } else if(gamepad1.dpad_right) {

            launchSpeed = launchSpeed - 0.05;
        }
        if(launchSpeed > 1) {
            launchSpeed = 1;
        }

        if(launchSpeed < 0.1) {
            launchSpeed = 0.1;
        }
        launch.setPower(launchSpeed);
        telemetry.addData("Launch Speed", launchSpeed);
        telemetry.update();
    }
}