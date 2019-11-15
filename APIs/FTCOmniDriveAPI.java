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
  double startFPosition;

  //Motors
  DcMotor leftMotor;
  DcMotor rightMotor;
  DcMotor strafeMotor;

  //Variables used to calulate distance per pulse
  private double DIAMETER = 4;
  private double RADIUS = DIAMETER/2;
  private double PULSESPERROTATION = 1120;
  private double circumference = 2*RADIUS*3.14159;
  double disPerPulse = circumference/PULSESPERROTATION;

  public FTCOmniDriveAPI(HardwareMap hwMap) {
    this.leftMotor = hwMap.get(DcMotor.class, "leftDrive");
    this.rightMotor = hwMap.get(DcMotor.class, "rightDrive");
    this.strafeMotor = hwMap.get(DcMotor.class, "strafeDrive");
  }


  //Creates the OmniDrive mathematical outputs for all motors based on an two x inputs and a y input or 3 values: Forward, Strafe, and Rotation
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

  //Cnotrol the chassis as an omni drive using joystick inputs
  public void driveOmniJoystick(float leftJoystickX, float leftJoystickY, float rightJoystickX) {
    calculateWheelSpeeds(leftJoystickX, -rightJoystickX, -leftJoystickY);
    this.rightMotor.setPower(rightMotorSpeed);
    this.leftMotor.setPower(leftMotorSpeed);
    this.strafeMotor.setPower(strafeMotorSpeed);
  }

  public void driveOmni(float xSpeed, float ySpeed, float rotateSpeed) {
    calculateWheelSpeeds(rotateSpeed, ySpeed, xSpeed);
    this.rightMotor.setPower(rightMotorSpeed);
    this.leftMotor.setPower(leftMotorSpeed);
    this.strafeMotor.setPower(strafeMotorSpeed);
  }

  //Allows you to control each part of the robot's chassis
  public void controlChassis(double leftPower, double rightPower, double strafePower) {
    this.rightMotor.setPower(rightPower);
    this.leftMotor.setPower(leftPower);
    this.strafeMotor.setPower(strafePower);
  }

  //Drives forward or backwards based on a power input infintely
  public void driveStraight(float straightPower) {
    driveOmni(straightPower,0,0);
  }

  //Drives forward or backward to a certain distance at a power
  public void driveStraight(float straightPower, double targetDistanceInch) {
    //Transform the encoder counts to start positions and finish positions
    this.startFPosition = getDistanceStraight();
    double totalDistance = getDistanceStraight() - this.startFPosition;

    if (totalDistance >= 0) {
      while (totalDistance >= 0) {
        driveOmni(straightPower,0,0);
        totalDistance = getDistanceStraight() - this.startFPosition;
      }
      stopMotors();
    } else {
      while (totalDistance <= 0) {
        if (straightPower > 0) {
          straightPower = -straightPower;
        }
        driveOmni(straightPower,0,0);
        totalDistance = getDistanceStraight() - this.startFPosition;
      }
      stopMotors();
    }
  }

  //strafes forward or backwards based on a power input infintely
  public void driveStrafe(float strafePower) {
    driveOmni(0,strafePower,0);
  }

  //strafes forward or backward to a certain distance at a power
  public void driveStrafe(float strafePower, float distanceInch) {
    driveOmni(0,strafePower,0);
  }

  //Rotates forward or backwards based on a power input infintely
  public void driveRotate(float rotatePower) {
    driveOmni(0,0,rotatePower);
  }

  //Rotates forward or backward to a angle at a power
  public void driveRotate(float rotatePower, float angle) {
    driveOmni(0,0,rotatePower);
  }

  private double getDistance(DcMotor motor) {
    return motor.getCurrentPosition() * this.disPerPulse;
  }
  //Stops all motors and sets them to 0
  public void stopMotors() {
    this.rightMotor.setPower(0);
    this.leftMotor.setPower(0);
    this.strafeMotor.setPower(0);
  }

  public double getDisPerPulse() {
    return this.disPerPulse;
  }

  public double getDistanceStraight() {
    return ((-getDistance(this.leftMotor) + getDistance(this.rightMotor))/2);
  }

  public double getDistanceStrafe() {
    return getDistance(this.strafeMotor);
  }

  public double getAngleRotate() {
    return 0;
  }
  public double getLeftMotorSpeed(){
    return this.leftMotorSpeed;
  }

  public double getRightMotorSpeed(){
    return this.rightMotorSpeed;
  }

  public double getStrafeMotorSpeed(){
    return this.strafeMotorSpeed;
  }
}
