package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.APIs.FTCOmniDriveAPI;
import org.firstinspires.ftc.teamcode.APIs.IntakeAPI;

@TeleOp(name="BishopKelleyQualifier")

public class BishopKelleyQualifier extends LinearOpMode {

    @Override
    public void runOpMode() {
      
        FTCOmniDriveAPI robot = new FTCOmniDriveAPI(hardwareMap, telemetry);
        IntakeAPI intake = new IntakeAPI(hardwareMap, telemetry);
        
        boolean aToggle = false; 
        boolean previousA = false;

        gamepad1.setJoystickDeadzone(0);

        telemetry.addLine("Robot Status : Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.driveOmniJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if(gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0){
              intake.controlElevator(gamepad1.left_trigger);
            } else if(gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0){
              intake.controlElevator(-gamepad1.right_trigger);
            } else {
              intake.controlElevator(0);
            }
  
            // if (gamepad1.a != previousA) {
            //   aToggle = !aToggle;
            // }
            // if (aToggle) {
            //   boolean finished = intake.setHeight(.4,5);
            //   if (!finished) {
            //     finished = intake.setHeight(.4,5);
            //   } else {
            //     intake.stopIntake();
            //   }
            // }
            // previousA = gamepad1.a;
            
            if(gamepad1.left_bumper && !gamepad1.right_bumper){
              intake.controlIntake(1);
            } else if(gamepad1.right_bumper && !gamepad1.left_bumper){
              intake.controlIntake(-1);
            } else {
              intake.controlIntake(0);
            }
        }
    }
}
