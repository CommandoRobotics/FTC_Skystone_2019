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
        this.hwMap = aHwMap;
        this.leftDriveMotor = this.hwMap.get(DcMotor.class, "leftDrive");
        this.rightDriveMotor = this.hwMap.get(DcMotor.class, "rightDrive");
        this.strafeDriveMotor = this.hwMap.get(DcMotor.class, "strafeDrive");
    }
    
    public boolean leftDriveMotorBusy(){
        return this.leftDriveMotor.isBusy();
    }
    
    public boolean rightDriveMotorBusy(){
        return this.rightDriveMotor.isBusy();
    }
    
    public boolean strafeDriveMotorBusy(){
        return this.strafeDriveMotor.isBusy();
    }
    
    public void resetEncoders(){
        this.leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.strafeDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void strafeToBuildingZone(){
        this.strafeDriveMotor.setTargetPosition(6500);
        this.strafeDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.strafeDriveMotor.setPower(0.75);
    }
    
    public void driveAwayFromWall(){
        this.leftDriveMotor.setTargetPosition(-3500);
        this.rightDriveMotor.setTargetPosition(3500);
        this.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftDriveMotor.setPower(0.75);
        this.rightDriveMotor.setPower(0.75);
    }
    
    public void stopMotors() {
        // this.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // this.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftDriveMotor.setPower(0);
        this.rightDriveMotor.setPower(0);
        this.strafeDriveMotor.setPower(0);
    }
    
    public void driveForward(double power) {
        this.leftDriveMotor.setPower(-power);
        this.rightDriveMotor.setPower(power);
    }
    public void controlDrive(double leftPower, double rightPower, double strafePower) {
        this.leftDriveMotor.setPower(leftPower);
        this.rightDriveMotor.setPower(rightPower);
        this.strafeDriveMotor.setPower(strafePower);
    }
    
}
