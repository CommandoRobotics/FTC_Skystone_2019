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

        FTCOmniDriveAPI robot = new FTCOmniDriveAPI(hardwareMap);
        IntakeAPI intake = new IntakeAPI(hardwareMap, telemetry);

        gamepad1.setJoystickDeadzone(0);

        telemetry.addLine("Robot Status : Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.driveOmniJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            //Elevator control logc
            if(gamepad1.left_bumper && !gamepad1.right_bumper){
              intake.controlElevator(-0.5);
            } else if(gamepad1.right_bumper && !gamepad1.left_bumper){
              intake.controlElevator(0.5);
            } else {
              intake.controlElevator(0);
            }

            if(gamepad1.left_trigger > 0.1 && !(gamepad1.right_trigger > 0.1)){
              intake.controlIntake(gamepad1.left_trigger);
            } else if(gamepad1.right_trigger > 0.1 && !(gamepad1.left_trigger > 0.1)){
              intake.controlIntake(gamepad1.right_trigger);
            } else {
              intake.controlIntake(0);
            }
        }
    }
}
