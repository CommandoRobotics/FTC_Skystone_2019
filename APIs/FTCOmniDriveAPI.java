package org.firstinspires.ftc.teamcode.APIs;

import java.lang.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.APIs.*;

public class FTCOmniDriveAPI{

  //Outputs to wheel
  double leftMotorSpeed;
  double rightMotorSpeed;
  double strafeMotorSpeed;
  double startFPosition;
  double startSPosition;
  double targetFPosition;
  double targetSPosition;

  //Motors
  DcMotor leftMotor;
  DcMotor rightMotor;
  DcMotor strafeMotor;
  Telemetry telemetry;
  GyroscopeAPI gyro;
  PidAPI leftStraightPID;
  PidAPI rightStraightPID;
  PidAPI strafeStraightPID;
  PidAPI leftRotatePID;
  PidAPI rightRotatePID;
  PidAPI strafeRotatePID;


  //Color Sensors
  ColorSensorAPI leftFColor;
  ColorSensorAPI rightFColor;
  public static ColorSensorAPI undersideColor;

  //Variables used to calulate distance per pulse
  private double DIAMETER = 4;
  private double RADIUS = DIAMETER/2;
  private double PULSESPERROTATION = 1120;
  private double circumference = 2*RADIUS*3.14159;
  double disPerPulse = circumference/PULSESPERROTATION;

  public FTCOmniDriveAPI(HardwareMap hwMap, Telemetry tele) {
    this.leftMotor = hwMap.get(DcMotor.class, "leftDrive");
    this.rightMotor = hwMap.get(DcMotor.class, "rightDrive");
    this.strafeMotor = hwMap.get(DcMotor.class, "strafeDrive");
    this.gyro = new GyroscopeAPI(hwMap);
    //this.leftStraightPID = new PidAPI();
    //this.rightStraightPID = new PidAPI();
    //this.strafeStraightPID = new PidAPI();
    this.leftRotatePID = new PidAPI(PidAPI.PID_MODE, 0.5, 0.02, 0.0006, 0.01, 1e9);
    this.rightRotatePID = new PidAPI(PidAPI.PID_MODE, 0.5, 0.02, 0.0006, 0.01, 1e9);
    leftRotatePID.makeAllGainsNegative();
    //this.strafeRotatePID = new PidAPI();
    // this.rightFColor = new ColorSensorAPI(hwMap, "frontRightColorSensor");
    // this.leftFColor = new ColorSensorAPI(hwMap, "frontLeftColorSensor");
    this.undersideColor = new ColorSensorAPI(hwMap, "undersideColor");
    this.strafeMotor.setDirection(DcMotor.Direction.REVERSE);
    this.telemetry = tele;
  }


  //Creates the OmniDrive mathematical outputs for all motors based on an two x inputs and a y input or 3 values: Forward, Strafe, and Rotation
  public void calculateWheelSpeeds(float joystick1x, float joystick1y, float joystick2x) {

    double MINIMUM_WHEEL_SPEED = .2;

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

    double forwardRotationDifference;
    if(forwardInput > rotationInput){
      forwardRotationDifference = forwardInput-rotationInput;
    } else if(forwardInput < rotationInput){
      forwardRotationDifference = rotationInput-forwardInput;
    } else {
      forwardRotationDifference = 0;
    }
    if(forwardRotationDifference <= MINIMUM_WHEEL_SPEED){
      if(forwardInput < 0){
        if(rotationInput < 0){
          targetRightMotorSpeed = MINIMUM_WHEEL_SPEED;
        } else if(rotationInput > 0){
          targetLeftMotorSpeed = MINIMUM_WHEEL_SPEED;
        }
      } else if(forwardInput > 0){
        if(rotationInput < 0){
          targetLeftMotorSpeed = MINIMUM_WHEEL_SPEED;
        } else if(rotationInput > 0){
          targetRightMotorSpeed = MINIMUM_WHEEL_SPEED;
        }
      }
    } else {
      leftMotorSpeed = targetLeftMotorSpeed;
      rightMotorSpeed = targetRightMotorSpeed;
    }
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
    calculateWheelSpeeds(xSpeed, ySpeed, rotateSpeed);
    this.rightMotor.setPower(rightMotorSpeed);
    this.leftMotor.setPower(-leftMotorSpeed);
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
    driveOmni(0,straightPower,0);
  }



  //DRIVE WITH JUST ENCODERS



  //Combines setTargetStraightPosition and straightWithEnc
  public void driveStraightEnc(double distance, float power) {
    setTargetStraightPosition(distance);
    telemetry.addLine("Starting to Drive Straight");
    while(!straightWithEnc(power)) {
      straightWithEnc(power);
      telemetry.update();
    }
    stopMotors();
  }

  //Sets the target distance for straightWithEnc
  public void setTargetStraightPosition(double targetDistanceInch) {
    this.targetFPosition = getDistanceStraight() + targetDistanceInch;
  }

  //Drives forward or backward to a certain distance at a power
  //meant to be run continuously
  public boolean straightWithEnc(float straightPower) {
    //Transform the encoder counts to start positions and finish positions
    double totalDistance = targetFPosition - getDistanceStraight();
    straightPower = Math.abs(straightPower);
    boolean finished = false;

    if(totalDistance >= 0) {
        if (totalDistance <= .1 && totalDistance >= -.1) {
          stopMotors();
          telemetry.addLine("driveSraightEnc() COMPLETE");
          finished = true;
        } else {
          driveOmni(0,straightPower,0);
          telemetry.addData("Distance Left to Drive: ", totalDistance);
          finished = false;
        }
    } else {
        if (totalDistance <= .1 && totalDistance >= -.1) {
          stopMotors();
          telemetry.addLine("driveSraightEnc() COMPLETE");
          finished = true;
        } else {
          driveOmni(0,-straightPower,0);
          telemetry.addData("Distance Left to Drive: ", totalDistance);
          finished = false;
        }
    }
    return finished;
  }

  //Combines setTargetStrafePosition and strafeWithEnc
  public void driveStrafeEnc(double distance, float power) {
    setTargetStrafePosition(distance);
    telemetry.addLine("Starting to Drive Strafe");
    while(!strafeWithEnc(power)) {
      strafeWithEnc(power);
      telemetry.update();
    }
    stopMotors();
  }

  //Sets the target distance for strafeWithEnc
  public void setTargetStrafePosition(double targetDistanceInch) {
    this.targetSPosition = getDistanceStrafe() + targetDistanceInch;
  }

  //Drives strafe to a certain distance at a power
  //meant to be run continuously
  public boolean strafeWithEnc(float strafePower) {
    //Transform the encoder counts to start positions and finish positions
    double totalDistance = targetSPosition - getDistanceStrafe();
    strafePower = Math.abs(strafePower);
    boolean finished = false;

    if(totalDistance >= 0) {
        if (totalDistance <= .1 && totalDistance >= -.1) {
          stopMotors();
          telemetry.addLine("driveStrafeEnc() COMPLETE");
          finished = true;
        } else {
          driveOmni(strafePower,0,0);
          telemetry.addData("Distance Left to Strafe: ", totalDistance);
          finished = false;
        }
    } else {
        if (totalDistance <= .1 && totalDistance >= -.1) {
          stopMotors();
          telemetry.addLine("driveStrafeEnc() COMPLETE");
          finished = true;
        } else {
          driveOmni(-strafePower,0,0);
          telemetry.addData("Distance Left to Strafe: ", totalDistance);
          finished = false;
        }
    }
    return finished;
  }



  //DRIVE WITH PID AND ENCODERS



  public void driveStraightPID(double distance, double bias) {

    setTargetStraightPosition(distance);
    telemetry.addLine("Starting to Drive Straight");

    double startTime = System.nanoTime();
    double dt = System.nanoTime() - startTime;
    gyro.update();
    double targetValue = -gyro.getZ();

    while(!straightWithPID(bias,dt,targetValue)) {
      dt = System.nanoTime() - startTime;
      straightWithPID(bias, dt,targetValue);
      telemetry.update();
    }
    stopMotors();
  }

  //Need to correct rotation, setPoint, and drift to left/right
  public boolean straightWithPID(double bias, double dt, double targetValue) {

    leftRotatePID.setBias(bias);
    rightRotatePID.setBias(bias);
    gyro.update();

    //Transform the encoder counts to start positions and finish positions
    double totalDistance = targetFPosition - getDistanceStraight();
    //straightPower = Math.abs(straightPower);
    boolean finished = false;

    if(totalDistance >= 0) {
        if (totalDistance <= .1 && totalDistance >= -.1) {
          stopMotors();
          telemetry.addLine("driveSraightPID() COMPLETE");
          finished = true;
        } else {
          leftMotor.setPower(-leftRotatePID.getOutput(-gyro.getZ(), targetValue, dt));
          rightMotor.setPower(rightRotatePID.getOutput(-gyro.getZ(), targetValue, dt));
          telemetry.addData("Distance Left to Drive: ", totalDistance);
          finished = false;
        }
    } else {
        if (totalDistance <= .1 && totalDistance >= -.1) {
          stopMotors();
          telemetry.addLine("driveSraightPID() COMPLETE");
          finished = true;
        } else {
          leftMotor.setPower(leftRotatePID.getOutput(-gyro.getZ(), targetValue, dt));
          rightMotor.setPower(-rightRotatePID.getOutput(-gyro.getZ(), targetValue, dt));
          telemetry.addData("Distance Left to Drive: ", totalDistance);
          finished = false;
        }
    }
    return finished;
  }

  public void pidTest(double bias, double targetValue, double startTime) {
        telemetry.addData("Left PID", 0);
    telemetry.update();
    while(true) {
    double dt = System.nanoTime() - startTime;


    leftRotatePID.setBias(bias);
    rightRotatePID.setBias(bias);
    gyro.update();

    leftMotor.setPower(-leftRotatePID.getOutput(-gyro.getZ(), targetValue, dt));
    rightMotor.setPower(rightRotatePID.getOutput(-gyro.getZ(), targetValue, dt));
    telemetry.addData("Right PID", rightRotatePID.getOutput(gyro.getZ(), targetValue, dt));
    telemetry.addData("Left PID", leftRotatePID.getOutput(gyro.getZ(), targetValue, dt));
    telemetry.addData("Rotation: ", -gyro.getZ());
    telemetry.update();
    }
  }




  //Rotates forward or backwards based on a power input infintely
  public void driveRotate(float rotatePower) {
    driveOmni(0,0,rotatePower);
  }

  //Rotates forward or backward to a angle at a power
  public void driveRotate(float rotatePower, double angle) {
    driveOmni(0,0,rotatePower);
  }

  public double getDistance(DcMotor motor) {
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

  public void resetEncoders() {
    this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

  public double getRawRotation(){
    gyro.update();
    return -gyro.getZ();
  }

  public double getRotation(){
    gyro.update();
    double rawAngle = -gyro.getZ();
    double reducedAngle;
    double modifiedAngle;
    if(Math.abs(rawAngle) > 360){
      reducedAngle = rawAngle%360;
    } else {
      reducedAngle = rawAngle;
    }

    if(reducedAngle < 0){
      modifiedAngle = 360-reducedAngle;
    } else {
      modifiedAngle = reducedAngle;
    }
    return modifiedAngle;
  }
}
