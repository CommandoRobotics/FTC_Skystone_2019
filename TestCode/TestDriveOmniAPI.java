/*
This program was developed on Grover and is not meant for other robotics, but rather as a reference.*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.APIs.FTCOmniDriveAPI;

@TeleOp(name="TestDriveOmniAPI")
@Disabled

public class TestDriveOmniAPI extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor leftDriveMotor = hardwareMap.get(DcMotor.class, "leftWheel");
        DcMotor rightDriveMotor = hardwareMap.get(DcMotor.class, "rightWheel");
        FTCOmniDriveAPI RIPSteve = new FTCOmniDriveAPI();
        
        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            RIPSteve.calculateWheelSpeeds(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            leftDriveMotor.setPower(RIPSteve.getLeftMotorSpeed());
            rightDriveMotor.setPower(-RIPSteve.getRightMotorSpeed());
            telemetry.addLine("Left Stick Y : " + gamepad1.left_stick_y);
            telemetry.addLine("Right stick X : " + gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
