package org.firstinspires.ftc.teamcode.MainTeleop;

import org.firstinspires.ftc.teamcode.APIs.ColorSensorAPI;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp

public class ColorSensorTesting extends LinearOpMode{

    //ColorSensor frontRightColor = hardwareMap.get(ColorSensor.class, "frontRightColor");



    @Override
    public void runOpMode(){
        ColorSensorAPI color = new ColorSensorAPI(hardwareMap, "undersideColor");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Red", color.getRed());
            telemetry.addData("Green", color.getGreen());
            telemetry.addData("Blue", color.getBlue());
            telemetry.addData("Yellow detected?", color.isYellow());
            telemetry.addData("Black detected?", color.isBlack());
            telemetry.update();
        }
    }

}
