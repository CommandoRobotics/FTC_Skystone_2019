package org.firstinspires.ftc.teamcode.MainAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.APIs.*;

@Autonomous(name="Run this one plz")

public class NavigateWithoutStrafe extends LinearOpMode {

    IntakeAPI intake;
    FTCOmniDriveAPI chassis;
    @Override
    public void runOpMode() {




        intake = new IntakeAPI(hardwareMap, telemetry);
        chassis = new FTCOmniDriveAPI(hardwareMap, telemetry);
        boolean finished = false;
        double startTime;

        waitForStart();

         startTime =  System.currentTimeMillis();
         while (((System.currentTimeMillis()) - startTime) < 2500) {
             chassis.driveOmni(0,0,.3f);
         }




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
