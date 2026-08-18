package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TestDrive {

    private DcMotor FLDrive;


    public void init(HardwareMap hwMap) {
        FLDrive = hwMap.get(DcMotor.class, "FLDrive");
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setMotorSpeed(double speed) {
        FLDrive.setPower(speed);
    }
}
