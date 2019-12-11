package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
  Telemetry telemetry;

  double elevatorUpPower = .5;
  double elevatorDownPower = -.5;
  double inPower = .6;
  double outPower = -.3;
  private final double MAXHEIGHT = 0;
  double currentHeight;
  double totalDistance;
 
  //values used for encoders
  private double DIAMETER = 2;
  private double RADIUS = DIAMETER/2;
  private double PULSESPERROTATION = 1120;
  private double circumference = 2*RADIUS*3.14159;
  double disPerPulse = circumference/PULSESPERROTATION;

  public IntakeAPI(HardwareMap hwMap, Telemetry tele) {
    this.leftElevatorMotor = hwMap.get(DcMotor.class, "leftElevator");
    this.rightElevatorMotor = hwMap.get(DcMotor.class, "rightElevator");
    this.leftIntakeMotor = hwMap.get(DcMotor.class, "leftIntake");
    this.rightIntakeMotor = hwMap.get(DcMotor.class, "rightIntake");
    this.inTouch = hwMap.get(TouchSensor.class, "intakeTouchSensor");
    this.telemetry = tele;

    this.leftElevatorMotor.setDirection(DcMotor.Direction.REVERSE);
    this.leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

    this.leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    resetElevatorEncoders();
  }

  //Allows you to control the Elevator by setting it to a + or - power
  public void controlElevator(double power) {
    if (calculateHeight() >= MAXHEIGHT && power > 0) {
      stopElevator();
    } else {
      leftElevatorMotor.setPower(power);
      rightElevatorMotor.setPower(power);
    }
  }

  //For later. The goal is to be able to set a height and the motors will move there using encoders. Pretty simple
  public boolean setHeight(double power, double targetHeight) {
    currentHeight = calculateHeight();
    totalDistance = targetHeight - currentHeight;
    power = Math.abs(power);
    boolean finished = false;
<<<<<<< Updated upstream
    if (targetHeight >= MAXHEIGHT) {
      stopElevator();
        finished = true;
        telemetry.addLine("setHeight() FAILED: Height set over max height");
    } else if (currentHeight == targetHeight || totalDistance < .5 || finished) {
      stopElevator();
      finished = true;
    } else if (currentHeight > targetHeight && totalDistance > .5 && !finished) {
      controlElevator(-power);
      finished = false;
    } else if (currentHeight < targetHeight && totalDistance > .5 && !finished) {
      controlElevator(power);
      finished = false;
    }
    
=======
    while (totalDistance < .1 || totalDistance > .1 || !finished) {
        if (targetHeight >= MAXHEIGHT) {
          stopElevator();
          finished = true;
          telemetry.addLine("setHeight() FAILED: Height set over max height");
        } else if (currentHeight == targetHeight || totalDistance < .5 || finished) {
          stopElevator();
        } else if (currentHeight > targetHeight && totalDistance > .5 && !finished) {
          controlElevator(-power);
          finished = false;
        } else if (currentHeight < targetHeight && totalDistance > .5 && !finished) {
          controlElevator(power);
          finished = false;
        }

>>>>>>> Stashed changes
      if (finished) {
          telemetry.addLine("setHeight() COMPLETE");
      } else {
          telemetry.addData("Target height: ", targetHeight);
          telemetry.addData("Current height: ", calculateHeight());
      }
    }
      return finished;
  }

  //Sets the Elevator all the way back to the ground using encoder values
  //(designed to be run on a toggle button)
  public boolean resetElevator() {
    double currentHeight = calculateHeight();
    boolean finished = false;
    if (currentHeight > .25) {
      elevatorDown();
      finished = false;
    } else if(currentHeight <= .25) {
      stopElevator();
      finished = true;
    }
    
    if (finished) {
        telemetry.addLine("Elevator Height Complete/Reached");
    } else {
        telemetry.addData("Target height: ", 0);
        telemetry.addData("Current height: ", calculateHeight());
    }
    return finished;
  }

  //Forces the elevator up at a certain power no matter the signage of the inupt
  public void elevatorUp() {
    if (calculateHeight() >= MAXHEIGHT) {
      stopElevator();
    } else {
      this.leftElevatorMotor.setPower(this.elevatorUpPower);
      this.rightElevatorMotor.setPower(this.elevatorUpPower);
    }
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

  //Reset encoder values for the elevator
  public void resetElevatorEncoders() {
    this.leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.leftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.rightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  //Calculate distance traveled by a motor using a motor's current pulses * disPerPulse
  private double calculateDistance(DcMotor motor) {
    return motor.getCurrentPosition()*disPerPulse;
  }

  //Calucalte height by averaging the two elevator motors' distance together
  private double calculateHeight() {
    return (-calculateDistance(this.leftElevatorMotor) + calculateDistance(this.rightElevatorMotor))/2; //might need to be made negative
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