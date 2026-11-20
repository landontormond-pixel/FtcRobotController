/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This file includes a teleop (driver-controlled) file for the Mecanum Drive goBILDA® StarterBot Base
 * Chassis/Intake for the 2026-2027 FIRST® Tech Challenge. It leverages a mecanum drive system for
 * robot mobility, one motor driving an intake roller, and two servos which pull elements out of corners.
 */

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends OpMode {

    // Declare OpMode members.
    private DcMotor LFDrive = null;
    private DcMotor RFDrive = null;
    private DcMotor LBDrive = null;
    private DcMotor RBDrive = null;
    private DcMotor Intake = null;
    private DcMotorEx Cannon = null;
    private CRServo LIntake = null;
    private CRServo RIntake = null;
    private CRServo Launcher = null;

    private Servo Flipper = null;

    // Set up a variable for each drive wheel to save power level for telemetry.
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;

    double CannonVelocity = 2000;

    boolean flipperToggleState = false; // false = position 0, true = position 1
    boolean flipperLastButtonState = false; // tracks previous B button state


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        LFDrive = hardwareMap.get(DcMotorEx.class, "FLDrive");
        RFDrive = hardwareMap.get(DcMotorEx.class, "FRDrive");
        LBDrive = hardwareMap.get(DcMotorEx.class, "BLDrive");
        RBDrive = hardwareMap.get(DcMotorEx.class, "BRDrive");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Cannon = hardwareMap.get(DcMotorEx.class, "Cannon");
        LIntake = hardwareMap.get(CRServo.class, "LIntake");
        RIntake = hardwareMap.get(CRServo.class, "RIntake");
        Launcher = hardwareMap.get(CRServo.class, "Launcher");
        Flipper = hardwareMap.get(Servo.class, "Flipper");

        Cannon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        LFDrive.setDirection(DcMotorEx.Direction.REVERSE);
        RFDrive.setDirection(DcMotorEx.Direction.FORWARD);
        LBDrive.setDirection(DcMotorEx.Direction.REVERSE);
        RBDrive.setDirection(DcMotorEx.Direction.FORWARD);
        Launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        LFDrive.setZeroPowerBehavior(BRAKE);
        RFDrive.setZeroPowerBehavior(BRAKE);
        LBDrive.setZeroPowerBehavior(BRAKE);
        RBDrive.setZeroPowerBehavior(BRAKE);
        Intake.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        LIntake.setPower(0);
        RIntake.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        RIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        Flipper.setPosition(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Direct mapping: true -> 1, false -> 0
        // flipper.setPosition(gamepad1.a ? 1 : 0);
        if (gamepad1.x) {
            intakePower = -1;
        } else {
            intakePower = 1;
        }

        // toggle the flipper
        boolean currentButtonState = gamepad1.b;

        if (currentButtonState && !flipperLastButtonState) {
            flipperToggleState = !flipperToggleState;
            Flipper.setPosition(flipperToggleState ? 0.43 : 0.0);
        }

        flipperLastButtonState = currentButtonState;
        // end of toggle the flipper

        if (gamepad1.leftBumperWasPressed()) {
            CannonVelocity -= 100;
        } else if (gamepad1.rightBumperWasPressed()) {
            CannonVelocity += 100;
        }

        // Limit velocity
        if (CannonVelocity > 3000) {
            CannonVelocity = 3000;
        } else if (CannonVelocity < 1000) {
            CannonVelocity = 1000;
        }

        Intake.setPower(intakePower);
        LIntake.setPower(intakePower);
        RIntake.setPower(intakePower);
        Launcher.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        Cannon.setVelocity(CannonVelocity);

        telemetry.addData("Target Velocity", CannonVelocity);
        telemetry.addData("Actual Velocity", Cannon.getVelocity());
        telemetry.addData("LF", leftFrontPower);
        telemetry.addData("RF", rightFrontPower);
        telemetry.addData("LB", leftBackPower);
        telemetry.addData("RB", rightBackPower);
        telemetry.addData("Flipper", Flipper.getPosition());
        telemetry.addData("Intake", intakePower);
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        LFDrive.setPower(0);
        RFDrive.setPower(0);
        LBDrive.setPower(0);
        RBDrive.setPower(0);

        Intake.setPower(0);
        Cannon.setPower(0);

        LIntake.setPower(0);
        RIntake.setPower(0);
        Launcher.setPower(0);
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        LFDrive.setPower(leftFrontPower);
        RFDrive.setPower(rightFrontPower);
        LBDrive.setPower(leftBackPower);
        RBDrive.setPower(rightBackPower);

    }
}