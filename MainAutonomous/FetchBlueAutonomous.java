package org.firstinspires.ftc.teamcode.MainAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.APIs.*;

@Autonomous(name="Fetch Blue")
//@Disabled

public class FetchBlueAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {


    //blue > 320
    //red > 350

        FTCOmniDriveAPI chassis = new FTCOmniDriveAPI(hardwareMap, telemetry);
      IntakeAPI intake = new IntakeAPI(hardwareMap, telemetry);
      chassis.resetEncoders();
      chassis.resetEncoders();
      intake.resetElevatorEncoders();
      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();
      boolean error = false;

      waitForStart();

      //forward
      chassis.driveStraightPID(28, 0.7f, opModeIsActive());

      intake.controlIntake(-1);
      chassis.driveStraightPID(10, .35f, opModeIsActive());

      //delay
      double startTime = System.currentTimeMillis();
      while ((System.currentTimeMillis() - startTime < 1000) && opModeIsActive()) {
        intake.controlIntake(-1);
      }

      chassis.driveStraightPID(-12, .7f, opModeIsActive());


      //right strafe
      intake.controlIntake(0);
      chassis.driveStrafePID(-72, 0.9f, opModeIsActive());

      //forward w/elevator lifting
      chassis.setTargetStraightPosition(12);
      telemetry.addLine("Starting to Drive Straight");
      while ((!chassis.straightWithEnc(0.65f) || !intake.setHeight(0.4, 2)) && opModeIsActive()) {
        chassis.straightWithEnc(0.65f);
        intake.setHeight(0.4,2);
        telemetry.update();
      }

      intake.controlIntake(1);
      startTime = System.currentTimeMillis();
      while ((System.currentTimeMillis() - startTime < 1000) && opModeIsActive()) {
        intake.controlIntake(1);
      }

      intake.stopElevator();
      chassis.stopMotors();


      //back up while lowering elevator and outaking
      chassis.setTargetStraightPosition(-7);
      telemetry.addLine("Starting to Drive Straight");
      while ((!chassis.straightWithEnc(0.6f) || !intake.resetElevator()) && opModeIsActive()) {
        chassis.straightWithEnc(0.6f);
        intake.resetElevator();
        intake.controlIntake(1);
        telemetry.update();
      }

      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();

      while ((chassis.undersideColor.getBlue() < 320) && opModeIsActive()) {
        chassis.driveOmni(.4f,0,0);
      }
      intake.stopIntake();
      intake.stopElevator();
      chassis.stopMotors();


        // while (opModeIsActive()) {
        //     telemetry.addData("Blue: ", FTCOmniDriveAPI.undersideColor.getBlue());
        //     telemetry.update();

        //     while(!(FTCOmniDriveAPI.undersideColor.getBlue() > 205 && FTCOmniDriveAPI.undersideColor.getBlue() < 225) && !finished) {
        //         telemetry.addData("Blue: ", FTCOmniDriveAPI.undersideColor.getBlue());
        //         telemetry.update();
        //         //finished = (FTCOmniDriveAPI.undersideColor.getRed() > 300);
        //         chassis.controlChassis(0,0,-.30);
        //     }
        //     chassis.stopMotors();
        //     finished = true;
    //     telemetry.addLine("Driving to Foundation");
    //   chassis.setTargetStrafePosition(5);
    //   finished = chassis.driveStrafeEnc(.4f);
    //   telemetry.update();

    //   while (!finished) {
    //     telemetry.addLine("Driving to Foundation");
    //     finished = chassis.driveStrafeEnc(.4f);
    //     telemetry.update();
    //    }
    }
}
