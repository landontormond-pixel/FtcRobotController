package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestDrive;
@TeleOp
public class DriveTest extends OpMode {
    TestDrive Drive = new TestDrive();

    @Override
    public void init() {
        Drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        //if(gamepad1.a) {
            Drive.setMotorSpeed(0.3);
        //}
    }
}
