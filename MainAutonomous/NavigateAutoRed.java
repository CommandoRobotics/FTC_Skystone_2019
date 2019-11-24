package org.firstinspires.ftc.teamcode.MainAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.APIs.*;

@Autonomous(name="NavigateAutoRed")
@Disabled

public class NavigateAutoRed extends LinearOpMode {

    IntakeAPI intake;
    FTCOmniDriveAPI chassis;
    double startTime;
    @Override
    public void runOpMode() {


        intake = new IntakeAPI(hardwareMap, telemetry);
        chassis = new FTCOmniDriveAPI(hardwareMap, telemetry);
        boolean finished = false;
        
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Red: ", FTCOmniDriveAPI.undersideColor.getRed());
            telemetry.update();
            
            while(FTCOmniDriveAPI.undersideColor.getRed() < 300 && !finished) {
                telemetry.addData("Red: ", FTCOmniDriveAPI.undersideColor.getRed());
                telemetry.update();
                //finished = (FTCOmniDriveAPI.undersideColor.getRed() > 300);
                chassis.controlChassis(0,0,.35);
            }
            chassis.stopMotors();
            finished = true;
        }
    }
}
