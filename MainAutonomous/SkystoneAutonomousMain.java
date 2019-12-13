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

    @Override
    public void runOpMode() {

      FTCOmniDriveAPI chassis = new FTCOmniDriveAPI(hardwareMap, telemetry);
      IntakeAPI intake = new IntakeAPI(hardwareMap, telemetry);
      ColorSensorAPI color = new ColorSensorAPI(hardwareMap, "frontRightColor");
      chassis.resetEncoders();
      chassis.resetEncoders();
      intake.resetElevatorEncoders();
      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();
      boolean error = false;

      waitForStart();

      //forward
      chassis.driveStraightEnc(14, 0.5f);

      //right strafe
      chassis.driveStrafeEnc(62, 0.5f);

      //forward w/elevator lifting
      chassis.setTargetStraightPosition(16);
      telemetry.addLine("Starting to Drive Straight");
      while (!chassis.straightWithEnc(0.3f) || !intake.setHeight(0.4, 5)) {
        chassis.straightWithEnc(0.3f);
        intake.setHeight(0.4, 5);
        telemetry.update();
      }

      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();

      //back up while lowering elevator and outaking
      chassis.setTargetStraightPosition(-16);
      telemetry.addLine("Starting to Drive Straight");
      while (!chassis.straightWithEnc(0.3f) || !intake.setHeight(0.4, 5)) {
        chassis.straightWithEnc(0.3f);
        intake.setHeight(0.4, 5);
        intake.controlIntake(0.4);
        telemetry.update();
      }

      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();

      //left strafe and fully lower
      while(!intake.setHeight(.3,0)){
        intake.setHeight(.3,0);
      }

      chassis.driveStrafeEnc(-62, 0.5f);

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
      chassis.driveStrafeEnc(4, 0.2f);
      distanceTraveled = chassis.getDistanceStrafe() - startDistance;

      //Check for errors
      if (!error) {

        //Drive straight into the skystone
        intake.controlIntake(1);
        chassis.driveStraightEnc(10, .4f);

        //Wait a bit to make sure we get the skystone
        double startTime = System.nanoTime();
        while (System.nanoTime() < 1500) {
          chassis.stopMotors();
        }

        //back up with intake still on
        chassis.driveStraightEnc(-10, .4f);

      }

      //Stop all Motors
      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();

      //strafe right back to where you started to look for skystone
      chassis.driveStrafeEnc(distanceTraveled, 0.5f);

      if (!error) {
        //strafe right to platform
        chassis.driveStrafeEnc(-62, .5f);

        //forward w/elevator lifting
        chassis.setTargetStraightPosition(16);
        telemetry.addLine("Starting to Drive Straight");
        while (!chassis.straightWithEnc(0.3f) || !intake.setHeight(0.4, 5)) {
          chassis.straightWithEnc(0.3f);
          intake.setHeight(0.4, 5);
          telemetry.update();
        }

        intake.stopIntake();
        intake.stopElevator();
        chassis.stopMotors();

        //back up while lowering elevator and outaking
        chassis.setTargetStraightPosition(-10);
        telemetry.addLine("Starting to Drive Straight");
        while (!chassis.straightWithEnc(0.3f) || !intake.setHeight(0.4, 5)) {
          chassis.straightWithEnc(0.3f);
          intake.setHeight(0.4, 5);
          intake.controlIntake(0.4);
          telemetry.update();
        }
      }
      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();

      //left or right strafe to centerline depending on if an error was reported
      if(error) {
        while (chassis.undersideColor.getRed() < 250) {
          chassis.driveOmni(.5f,0,0);
        }
        intake.stopIntake();
        intake.stopElevator();
        chassis.stopMotors();
      } else {
        while (chassis.undersideColor.getRed() < 250) {
          chassis.driveOmni(-.5f,0,0);
        }
        intake.stopIntake();
        intake.stopElevator();
        chassis.stopMotors();
      }


      while (chassis.undersideColor.getRed() < 250) {
        chassis.driveOmni(-.5f,0,0);
      }
      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();
    }
}
