package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.APIs.FTCOmniDriveAPI;
import org.firstinspires.ftc.teamcode.APIs.VisionTrackingAPI;

@TeleOp(name="TeleopMain")

public class TeleopMain extends LinearOpMode {

VisionTrackingAPI vision = new VisionTrackingAPI();

    @Override
    public void runOpMode() {
        TouchSensor intakeButton = hardwareMap.get(TouchSensor.class, "intakeButton");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        
        FTCOmniDriveAPI RIPSteve = new FTCOmniDriveAPI(hardwareMap);
        
        gamepad1.setJoystickDeadzone(0);
        
        
        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();
        
        vision.initialize(hardwareMap);

        while (opModeIsActive()) {
            RIPSteve.driveOmni(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            
            // telemetry.addData("Intake button pressed" , intakeButton.isPressed());
            // telemetry.addData("Color sensor red value" , colorSensor.red());
            // telemetry.addData("Color sensor green value" , colorSensor.green());
            // telemetry.addData("Color sensor blue value" , colorSensor.blue());
            vision.searchForTargets();
            // telemetry.addData("Robot X", vision.getX());
            // telemetry.addData("Robot Y", vision.getY());
            // telemetry.addData("Robot Z", vision.getZ());
            // telemetry.addData("Robot Roll", vision.getRoll());
            // telemetry.addData("Robot Pitch", vision.getPitch());
            // telemetry.addData("Robot Heading", vision.getHeading());
            
            telemetry.update();
            
        }
    }
}