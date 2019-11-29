//This program assumes you are starting on the building side from the red alliance side

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.APIs.*;

@Autonomous(name="Test Autonomous")

public class SkystoneAutonomousMain extends LinearOpMode {

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
