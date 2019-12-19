// /*

// This program was developed on Grover and is not meant for other robotics, but rather as a reference.*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.APIs.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="NewRobotTest")
//@Disabled

public class NewRobotTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        FTCOmniDriveAPI robot = new FTCOmniDriveAPI(hardwareMap, telemetry);
        IntakeAPI intake = new IntakeAPI(hardwareMap, telemetry);

        telemetry.addLine("Robot initialized");
        telemetry.update();
        robot.resetEncoders();

        boolean previousA = false;
        boolean elevatorActive = false;

        waitForStart();

        while (opModeIsActive()) {
          if(gamepad1.a && gamepad1.a != previousA){
              elevatorActive = !elevatorActive;
            }
            previousA = gamepad1.a;

            if (elevatorActive) {
                intake.setHeight(.3,2);
                if (intake.setHeight(.3,2)) {
                    //elevatorActive = false;
                }
            }
            telemetry.addData("red", robot.rightFColor.getRed());
            telemetry.addData("green`", robot.rightFColor.getGreen());
            telemetry.addData("blue", robot.rightFColor.getBlue());

            telemetry.update();
        }





    }
}
