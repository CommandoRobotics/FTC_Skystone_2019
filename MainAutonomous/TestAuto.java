package org.firstinspires.ftc.teamcode.MainAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.APIs.*;

@Autonomous(name="TestAuto")
@Disabled

public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
      FTCOmniDriveAPI chassis = new FTCOmniDriveAPI(hardwareMap, telemetry);
      boolean finished;
      telemetry.addLine("Driving to Foundation");
      chassis.setTargetStraightPosition(11);
      finished = chassis.driveStraightEnc(.2f);
      telemetry.update();
      
      while (!finished) {
        telemetry.addLine("Driving to Foundation");
        finished = chassis.driveStraightEnc(.2f);
        telemetry.update();
      }
    }
}