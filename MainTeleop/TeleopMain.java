package org.firstinspires.ftc.teamcode.MainTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleopMain")

public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode(){

        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

        }

    }
}
