package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.TestCode.HDrive;

@Autonomous(name="TestAutonomous")

public class TestAutonomous extends LinearOpMode{
    
    @Override
    public void runOpMode(){
        HDrive robot = new HDrive(hardwareMap);
        telemetry.addLine("Status : Initialized");
        telemetry.update();
        
        waitForStart();
        
        while(opModeIsActive()){
            robot.driveForwardToPosition();
        }
    }
}