//This program assumes you are starting on the building side from the red alliance side

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.APIs.*;

@Autonomous(name="Red Autonomous")

public class SkystoneAutonomousMain extends LinearOpMode {
  
  //Pathing for RED auto assuming NO pulling platform
  //Initialize
  chassis.resetEncoders();
  intake.resetElevatorEncoders();
  intake.stopIntake();
  intake.stopElevator();
  chassis.stopMotors();
  boolean error = false;
  
  //forward
  chassis.driveStraightEnc(0.5f, 14);
  
  //right strafe
  chassis.driveStrafeEnc(0.5f, 62);
  
  //forward w/elevator lifting
  chassis.setTargetStraightPosition(16);
  telemetry.addLine("Starting to Drive Straight");
  while (!chassis.straightWithENC(0.3f) || !intake.setHeight(0.4, 5)) {
    chassis.straightWithENC(0.3f);
    intake.setHeight(0.4, 5);
    telemetry.update();
  }
  
  stopAllMotors();
  
  //back up while lowering elevator and outaking
  chassis.setTargetStraightPosition(-16);
  telemetry.addLine("Starting to Drive Straight");
  while (!chassis.straightWithENC(0.3f) || !intake.setHeight(0.4, 5)) {
    chassis.straightWithENC(0.3f);
    intake.setHeight(0.4, 5);
    intake.controlIntake(0.4);
    telemetry.update();
  }
  
  stopAllMotors();
  
  //left strafe and fully lower
  intake.resetElevator();
  chassis.driveStrafeEnc(0.5f, -62);
  
  //strafe left slowly until see black
  double startDistance = chassis.getDistanceStrafe();
  double distanceTraveled = 0;
  
  //Go slowly left until see black
  while ((!chassis.rightFColor.isBlack() && chassis.rightFColor.isYellow()) && !error) {
    chassis.driveOmni(-.3f,0,0);
    
    if (chassis.rightFColor.isBlack() == true && chassis.rightFColor.isYellow() == true) {
      telemetry.addLine("Color Sensor Error: Detected both black and yellow");
      telemetry.update();
      error = true;
    }
  }
  chassis.stopMotors();
  
  //Go back right a bit to until no black seen and then center yourself
  while ((chassis.rightFColor.isBlack() && !chassis.rightFColor.isYellow()) && !error) {
    chassis.driveOmni(.2f,0,0);
    
    if (chassis.rightFColor.isBlack() == true && chassis.rightFColor.isYellow() == true) {
      telemetry.addLine("Color Sensor Error: Detected both black and yellow");
      telemetry.update();
      error = true;
    }
  }
  chassis.stopMotors();
  
  //Now center by driving right a bit more and record distanceTraveled
  chassis.driveStrafeEnc(0.2f, 4);
  distanceTraveled = chassis.getDistanceStrafe() - startDistance;
  
  //Check for errors
  if (!error) {
    
    //Drive straight into the skystone
    intake.controlIntake(1);
    chassis.driveStraightEnc(.4f, 10);
    
    //Wait a bit to make sure we get the skystone
    double startTime = system.nanoTime();
    while (system.nanoTime() < 1500) {
      chassis.stopMotors();
    }
    
    //back up with intake still on 
    chassis.driveStraightEnc(.4f, -10);

  }
  
  //Stop all Motors
  stopAllMotors();
  
  //strafe right back to where you started to look for skystone
  chassis.driveStrafeEnc(.5f, distanceTraveled);
  
  //strafe right to platform
  chassis.driveStrafeEnc(.5f, -62);
  
  //forward w/elevator lifting
  chassis.setTargetStraightPosition(16);
  telemetry.addLine("Starting to Drive Straight");
  while (!chassis.straightWithENC(0.3f) || !intake.setHeight(0.4, 5)) {
    chassis.straightWithENC(0.3f);
    intake.setHeight(0.4, 5);
    telemetry.update();
  }
  
  stopAllMotors();
  
  //back up while lowering elevator and outaking
  chassis.setTargetStraightPosition(-10);
  telemetry.addLine("Starting to Drive Straight");
  while (!chassis.straightWithENC(0.3f) || !intake.setHeight(0.4, 5)) {
    chassis.straightWithENC(0.3f);
    intake.setHeight(0.4, 5);
    intake.controlIntake(0.4);
    telemetry.update();
  }
  
  stopAllMotors();
  
  //left strafe to centerline
  while (chassis.undersideColor.red() < 250) {
    chassis.driveOmni(-.5f,0,0);
  }
  stopAllMotors();
    
  public void stopAllMotors() {
    intake.stopIntake();
    intake.stopElevator();
    chassis.stopMotors();
  }
    

    @Override
    public void runOpMode() {

      FTCOmniDriveAPI chassis = new FTCOmniDriveAPI(hardwareMap, telemetry);
      IntakeAPI intake = new IntakeAPI(hardwareMap, telemetry);
      ColorSensorAPI color = new ColorSensorAPI(hardwareMap, "frontRightColor");
      chassis.resetEncoders();
      waitForStart();

      //Move elevator up
      intake.setHeight(0.5, 2);

      //Drive from wall to front of platform
      chassis.driveStraightEnc(30,0.5f);

      //Drive sideway to platform
      chassis.driveStrafeEnc(10,0.5f);

      //Place block
      intake.intakeOut();

      //Wait 1 second
      sleep(1000);

      //Stop intake
      intake.stopIntake();

      //DROP GRABBER

      //Move platform
      chassis.driveStraightEnc(-27,0.5f);

      //LIFT GRABBER

      //Strafe left
      chassis.driveStrafeEnc(-58,0.75f);

      //Raise elevator above blocks
      intake.setHeight(0.5, 5);

      //Drive towards quarry
      chassis.driveStraightEnc(30,0.5f);

      //Drive left while scanning for skystone
      while(color.isSkystoneDetected() == false){
        chassis.controlChassis(0,0,.4);
      }
      chassis.controlChassis(0,0,0);
    }
}
