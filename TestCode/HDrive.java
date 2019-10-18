package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class HDrive {
    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;
    DcMotor strafeDriveMotor;
    HardwareMap hwMap;

    public HDrive(HardwareMap aHwMap){
        hwMap = aHwMap;
        DcMotor leftDriveMotor = hwMap.get(DcMotor.class, "leftDrive");
        DcMotor rightDriveMotor = hwMap.get(DcMotor.class, "rightDrive");
        DcMotor strafeDriveMotor = hwMap.get(DcMotor.class, "strafeDrive");
    }

    public void driveForwardToPosition(){
        DcMotor leftDriveMotor = hwMap.get(DcMotor.class, "leftDrive");
        DcMotor rightDriveMotor = hwMap.get(DcMotor.class, "rightDrive");
        DcMotor strafeDriveMotor = hwMap.get(DcMotor.class, "strafeDrive");
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafeDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDriveMotor.setPower(0.75);
        rightDriveMotor.setPower(0.75);
        leftDriveMotor.setTargetPosition(100);
        rightDriveMotor.setTargetPosition(100);
    }
}
