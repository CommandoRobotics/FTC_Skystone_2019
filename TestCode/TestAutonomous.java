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
        //robot.resetEncoders();
        
        telemetry.addLine("Status : Initialized");
        telemetry.update();
        
        waitForStart();
        
        while(opModeIsActive()){
            // robot.strafeToBuildingZone();
            // while(robot.strafeDriveMotorBusy()){
                
            // }
            // robot.driveAwayFromWall();
            // while(robot.leftDriveMotorBusy()){
                
            // }
            robot.controlDrive(.75,-.75,0);
            sleep(1000);
            robot.stopMotors();
            robot.controlDrive(0,0,.75);
            sleep(1000);
            robot.stopMotors();
            robot.controlDrive(.75,.75,0);
            sleep(1000);
            robot.stopMotors();
        }
    }
}