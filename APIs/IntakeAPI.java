package org.firstinspires.ftc.teamcode.APIs;

//import java.lang.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeAPI{

  DcMotor leftElevatorMotor;
  DcMotor rightElevatorMotor;
  DcMotor leftIntakeMotor;
  DcMotor rightIntakeMotor;
  TouchSensor inTouch;

  double elevatorUpPower = .5;
  double elevatorDownPower = -.5;
  double inPower = .6;
  double outPower = -.3;

  public IntakeAPI(HardwareMap hwMap) {
    this.leftElevatorMotor = hwMap.get(DcMotor.class, "leftElevator");
    this.rightElevatorMotor = hwMap.get(DcMotor.class, "rightElevator");
    this.leftIntakeMotor = hwMap.get(DcMotor.class, "leftIntake");
    this.rightIntakeMotor = hwMap.get(DcMotor.class, "rightIntake");
    this.inTouch = hwMap.get(TouchSensor.class, "intakeTouchSensor");

    this.leftElevatorMotor.setDirection(DcMotor.Direction.REVERSE);
    this.leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

    this.leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  //Allows you to control the Elevator by setting it to a + or - power
  public void controlElevator(double power) {
    this.leftElevatorMotor.setPower(power);
    this.rightElevatorMotor.setPower(power);
  }

  //For later. The goal is to be able to set a height and the motors will move there using encoders. Pretty simple
  //public void setHeight(double power, double targetHeight) {
  //
  //}

  //Sets the Elevator all the way back to the ground using encoder values
  //public void resetElevator(double power) {
  //
  //}

  //Forces the elevator up at a certain power no matter the signage of the inupt
  public void elevatorUp() {
    this.leftElevatorMotor.setPower(this.elevatorUpPower);
    this.rightElevatorMotor.setPower(this.elevatorUpPower);
  }

  ////Forces the elevator down at a certain power no matter the signage of the inupt
  public void elevatorDown() {
    this.leftElevatorMotor.setPower(this.elevatorDownPower);
    this.rightElevatorMotor.setPower(this.elevatorDownPower);
  }

  //Stops all Elevator motors
  public void stopElevator() {
    this.leftElevatorMotor.setPower(0);
    this.rightElevatorMotor.setPower(0);
  }

  //Controls the Intake motors to run at a certain power either + (in) or - (out)
  public void controlIntake(double power) {
    if (this.inTouch.isPressed() && power>0) {
        this.leftIntakeMotor.setPower(0);
        this.rightIntakeMotor.setPower(0);
    } else {
        this.leftIntakeMotor.setPower(power);
        this.rightIntakeMotor.setPower(power);
    }
  }

  //Runs the Intake at a set speed inwards (collecting)
  public void intakeIn() {
    if (this.inTouch.isPressed()) {
        this.leftIntakeMotor.setPower(0);
        this.rightIntakeMotor.setPower(0);
    } else {
        this.leftIntakeMotor.setPower(this.inPower);
        this.rightIntakeMotor.setPower(this.inPower);
    }
  }

  //Runs the Intake at a set speed outwards (scoring)
  public void intakeOut() {
    this.leftIntakeMotor.setPower(this.outPower);
    this.rightIntakeMotor.setPower(this.outPower);
  }

  //Stops all Intake Motors
  public void stopIntake() {
    this.leftIntakeMotor.setPower(0);
    this.rightIntakeMotor.setPower(0);
  }

  //Sets the Elevator up speed if needed
  public void setElevatorUpPower(double power) {
    this.elevatorUpPower = power;
  }

  //Sets the elevator down speed if needed
  public void setElevatorDownPower(double power) {
    this.elevatorDownPower = power;
  }

  //Returns the set Elevator up power
  public double getElevatorUpPower() {
    return this.elevatorUpPower;
  }

  //Returns the set Elevator down power
  public double getElevatorDownPower() {
    return this.elevatorDownPower;
  }

  //Returns the set Intake in power
  public double getIntakeInPower() {
    return this.inPower;
  }

  //Returns the set Intake out power
  public double getIntakeOutPower() {
    return this.outPower;
  }

  //Tells us if a stone is in the intake
  public boolean isStoneInIntake() {
    return this.inTouch.isPressed();
  }
}