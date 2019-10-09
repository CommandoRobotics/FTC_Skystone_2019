/*
This program was developed on Grover and is not meant for other robotics, but rather as a reference.*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TestButton")
@Disabled

public class TestButton extends LinearOpMode {

    @Override
    public void runOpMode() {

        // public TouchSensor testButton();
        TouchSensor testButton = hardwareMap.get(TouchSensor.class , "testButton");
        DcMotor testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Is button pressed?", testButton.getValue());

            if(testButton.getValue() == 1){
              testMotor.setPower(1);
            } else {
                testMotor.setPower(0);
            }

            telemetry.update();
        }
    }
}
