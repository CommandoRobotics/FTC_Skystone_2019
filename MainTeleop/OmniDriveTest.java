package org.firstinspires.ftc.teamcode.MainTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.APIs.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="OmniDriveTest")

public class OmniDriveTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        FTCOmniDriveAPI robot = new FTCOmniDriveAPI(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()){
            robot.driveOmniJoystick(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Left stick y", -gamepad1.left_stick_y);
            telemetry.addData("Right stick x", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
