package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.APIs.FTCOmniDriveAPI;

@TeleOp(name="TeleopMain")

public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        TouchSensor intakeButton = hardwareMap.get(TouchSensor.class, "intakeButton");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        
        FTCOmniDriveAPI RIPSteve = new FTCOmniDriveAPI(hardwareMap);
        
        gamepad1.setJoystickDeadzone(0);
        
        
        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            RIPSteve.driveOmni(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            
            telemetry.addData("Intake button pressed" , intakeButton.isPressed());
            telemetry.addData("Color sensor red value" , colorSensor.red());
            telemetry.addData("Color sensor green value" , colorSensor.green());
            telemetry.addData("Color sensor blue value" , colorSensor.blue());
            
            telemetry.update();
            
        }
    }
}