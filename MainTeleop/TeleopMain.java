package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.APIs.FTCOmniDriveAPI;
import org.firstinspires.ftc.teamcode.APIs.IntakeAPI;
import org.firstinspires.ftc.teamcode.APIs.CapstonePlacerAPI;

@TeleOp(name="TeleopMain")

public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() {

        FTCOmniDriveAPI robot = new FTCOmniDriveAPI(hardwareMap, telemetry);
        IntakeAPI intake = new IntakeAPI(hardwareMap, telemetry);
        CapstonePlacerAPI capstonePlacer = new CapstonePlacerAPI(hardwareMap);

        intake.resetElevatorEncoders();

        boolean previousA = false;
        boolean slowModeActive = false;

        gamepad1.setJoystickDeadzone(0);

        capstonePlacer.initialize();

        telemetry.addLine("Robot Status : Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("", slowModeActive);
            telemetry.update();

            if(gamepad1.a && gamepad1.a != previousA){
              slowModeActive = !slowModeActive;
            }
            previousA = gamepad1.a;
            if(!slowModeActive){
              robot.driveOmniJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
              if(gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0){
                intake.controlElevatorS(gamepad1.left_trigger);
              } else if(gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0){
                intake.setHeight(.5,2.3);
              } else {
                intake.controlElevator(0);
              }

              if(gamepad1.left_bumper && !gamepad1.right_bumper){
                intake.controlIntake(1);
              } else if(gamepad1.right_bumper && !gamepad1.left_bumper){
                intake.controlIntake(-1);
              } else {
                intake.controlIntake(0);
              }
            } else {
              robot.driveOmniJoystick(gamepad1.left_stick_x*0.5f, gamepad1.left_stick_y*0.5f, gamepad1.right_stick_x*0.5f);
              if(gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0){
                intake.controlElevator(gamepad1.left_trigger);
              } else if(gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0){
                intake.controlElevator(-gamepad1.right_trigger);
              } else {
                intake.controlElevator(0);
              }

              if(gamepad1.left_bumper && !gamepad1.right_bumper){
                intake.controlIntake(0.3);
                robot.driveStraight(-0.5f);
              } else if(gamepad1.right_bumper && !gamepad1.left_bumper){
                intake.controlIntake(-1);
              } else {
                intake.controlIntake(0);
              }
            }

            if(gamepad1.b && gamepad1.dpad_left){
              capstonePlacer.place();
            }

            telemetry.addData("intake height: ", intake.calculateHeight());

        }
    }
}
