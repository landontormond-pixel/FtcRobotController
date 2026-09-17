package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;


@TeleOp(name = "Mecanum Fast Reload")
public class MecanumFastReload extends OpMode{
    DcMotor BR;
    DcMotor BL;
    DcMotor FL;
    DcMotor FR;
    DcMotorEx launch;
    DcMotor intake;
    CRServo Gate1;
    CRServo Gate2;
    CRServo intakeLeft;
    CRServo intakeRight;
    Servo LED;
    AnalogInput distanceSensor;

    double launchVelocity;
    @Override
    public void init () {

        //Variables



        //Motor Setup
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        intake = hardwareMap.get(DcMotor.class, "intake");

        //Servo Setup
        Gate1 = hardwareMap.get(CRServo.class, "Gate1");
        Gate2 = hardwareMap.get(CRServo.class, "Gate2");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        LED = hardwareMap.get(Servo.class, "light");

        //Analog Input
        distanceSensor = hardwareMap.get(AnalogInput.class, "distanceSensor");


        //Set Motor and Servo Direction
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        Gate1.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        launchVelocity = 1600;
        launch.setVelocity(launchVelocity);
        launch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void setRed() {
        LED.setPosition(0.2783333333333333);
    }
    private void setGreen() {
        LED.setPosition(0.5);
    }

    private void setBlue() {
        LED.setPosition(0.666);
    }

    @Override
    public void loop() {

        //Power for intakeLeft, intakeRight, intake, Gate1, and Gate2

        if(gamepad1.a) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        } else {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        }

        if(gamepad1.b) {
            intake.setPower(1);
        } else if(gamepad1.x) {
            intake.setPower(-1);
            intakeRight.setPower(-1);
            intakeLeft.setPower(-1);
        } else {
            intake.setPower(0);
        }
        if(gamepad1.y) {
            Gate1.setPower(-1);
            Gate2.setPower(-1);
        } else if(gamepad1.right_bumper) {
            Gate1.setPower(1);
            Gate2.setPower(1);
        } else {
            Gate1.setPower(0);
            Gate2.setPower(0);
        }


// JOYSTICK INPUTS

        double forward = gamepad1.left_stick_y;
        double sideways = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

// MECANUM DRIVE MATH

        double flPower = forward + sideways + rotate;
        double frPower = forward - sideways - rotate;
        double blPower = forward - sideways + rotate;
        double brPower = forward + sideways - rotate;

// APPLY POWER

        double driveScale = gamepad1.right_trigger > 0.1 ? 0.4 : 1.0;

        flPower *= driveScale;
        frPower *= driveScale;
        blPower *= driveScale;
        brPower *= driveScale;

        FL.setPower(flPower);
        FR.setPower(frPower);
        BL.setPower(blPower);
        BR.setPower(brPower);


        final double velocityTolerance = 75;
        final double smallTriVelocity = 1800;
        final double largeTriVelocity = 1600;
        double actualVelocity = launch.getVelocity();
        if (Math.abs(actualVelocity - largeTriVelocity) < velocityTolerance) {
            setGreen();
        } else if (Math.abs(actualVelocity - smallTriVelocity) < velocityTolerance) {
            setBlue();
        } else {
            setRed();
        }
        //Launch Speed Set
        if (gamepad1.left_bumper) {
            launchVelocity = smallTriVelocity;

        } else if(gamepad1.left_trigger > 0.1) {
            launchVelocity = largeTriVelocity;
        }


//SET POWER FOR BIG TRI USING DISTANCE SENSOR

        launch.setVelocity(launchVelocity);
        telemetry.addData("Launch Target Velocity (TPS)", launchVelocity);
        telemetry.addData("Actual Launch Velocity (TPS)", actualVelocity);
        telemetry.update();
    }
}