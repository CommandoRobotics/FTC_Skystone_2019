package org.firstinspires.ftc.teamcode.APIs;

import java.lang.*;

public class FTCOmniDriveAPI{

  //Outputs to wheel
  double leftMotorSpeed;
  double rightMotorSpeed;
  double strafeMotorSpeed;

  public void calculateWheelSpeeds(float joystick1x, float joystick1y, float joystick2x){
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

    if(Math.abs(targetLeftMotorSpeed) > 1){
      goldenRatio = targetLeftMotorSpeed/targetRightMotorSpeed;
      targetRightMotorSpeed = targetRightMotorSpeed/goldenRatio;
      if(targetLeftMotorSpeed > 1){
        targetLeftMotorSpeed = 1;
      } else if(targetLeftMotorSpeed < -1){
        targetLeftMotorSpeed = -1;
      }
    } else if(Math.abs(targetRightMotorSpeed) > 1){
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
