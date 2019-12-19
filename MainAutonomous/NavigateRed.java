package org.firstinspires.ftc.teamcode.MainAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.APIs.*;

@Autonomous(name="Red With Strafe")

public class NavigateRed extends LinearOpMode {

    IntakeAPI intake;
    FTCOmniDriveAPI chassis;
    @Override
    public void runOpMode() {


        FTCOmniDriveAPI chassis = new FTCOmniDriveAPI(hardwareMap, telemetry);
        IntakeAPI intake = new IntakeAPI(hardwareMap, telemetry);
        intake.resetElevatorEncoders();

        waitForStart();

        chassis.driveStraightPID(28,.5f, opModeIsActive());

        while ((chassis.undersideColor.getRed() < 350) && opModeIsActive()) {
            chassis.driveOmni(.4f,0,0);
        }
        intake.stopIntake();
        intake.stopElevator();
        chassis.stopMotors();

        // double startTime = System.currentTimeMillis();
        // while (System.currentTimeMillis()-startTime < 300) {
        //     intake.controlElevatorS(.5);
        //     telemetry.update();
        // }
        // intake.stopElevator();


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
