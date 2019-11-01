package org.firstinspires.ftc.teamcode.APIs;

import java.lang.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FTCOmniDriveAPI{

  //Outputs to wheel
  double leftMotorSpeed;
  double rightMotorSpeed;
  double strafeMotorSpeed;
  
  DcMotor leftMotor;
  DcMotor rightMotor;
  DcMotor strafeMotor;
  
  public FTCOmniDriveAPI(HardwareMap hwMap) {
    this.leftMotor = hwMap.get(DcMotor.class, "leftDrive");
    this.rightMotor = hwMap.get(DcMotor.class, "rightDrive");
    this.strafeMotor = hwMap.get(DcMotor.class, "strafeDrive");
  }
  
  

  public void calculateWheelSpeeds(float joystick1x, float joystick1y, float joystick2x) {
    //create 3D inputs based on joystick coordinates
    double forwardInput = (double) joystick1y;
    double strafeInput = (double) joystick1x;
    double rotationInput = -(double) joystick2x;

    double targetLeftMotorSpeed;
    double targetRightMotorSpeed;
    double targetStrafeMotorSpeed;
    double goldenRatio;

    targetLeftMotorSpeed = rotationInput+forwardInput;
    targetRightMotorSpeed = -rotationInput+forwardInput;
    targetStrafeMotorSpeed = strafeInput;

    if(Math.abs(targetLeftMotorSpeed) > 1) {
      goldenRatio = targetLeftMotorSpeed/targetRightMotorSpeed;
      targetRightMotorSpeed = targetRightMotorSpeed/goldenRatio;
      if(targetLeftMotorSpeed > 1) {
        targetLeftMotorSpeed = 1;
      } else if(targetLeftMotorSpeed < -1) {
        targetLeftMotorSpeed = -1;
      }
    } else if(Math.abs(targetRightMotorSpeed) > 1) {
      goldenRatio = targetRightMotorSpeed/targetLeftMotorSpeed;
      targetLeftMotorSpeed = targetLeftMotorSpeed/goldenRatio;
      if(targetRightMotorSpeed > 1){
        targetRightMotorSpeed = 1;
      } else if(targetRightMotorSpeed < -1){
        targetRightMotorSpeed = -1;
      }
    }

    leftMotorSpeed = targetLeftMotorSpeed;
    rightMotorSpeed = targetRightMotorSpeed;
    strafeMotorSpeed = targetStrafeMotorSpeed;

  }
  
  public void driveOmni(float leftJoystickX, float leftJoystickY, float rightJoystickX) {
    calculateWheelSpeeds(leftJoystickX, -rightJoystickX, -leftJoystickY);
    this.rightMotor.setPower(rightMotorSpeed);
    this.leftMotor.setPower(leftMotorSpeed);
    this.strafeMotor.setPower(strafeMotorSpeed);
  }
  
  public void controlChassis(double leftPower, double rightPower, double strafePower) {
    this.rightMotor.setPower(rightPower);
    this.leftMotor.setPower(leftPower);
    this.strafeMotor.setPower(strafePower);
  }
  
  public void stopMotors() {
    this.rightMotor.setPower(0);
    this.leftMotor.setPower(0);
    this.strafeMotor.setPower(0);
  }
  
  
  
  public double getLeftMotorSpeed(){
    return leftMotorSpeed;
  }

  public double getRightMotorSpeed(){
    return rightMotorSpeed;
  }

  public double getStrafeMotorSpeed(){
    return strafeMotorSpeed;
  }
}
