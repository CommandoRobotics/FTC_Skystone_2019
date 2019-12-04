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
        telemetry.addLine("Robot initialized");
        telemetry.update();
        robot.resetEncoders();



        waitForStart();

        while (opModeIsActive()) {
            //telemetry.addData("LeftMotorDistance ", robot.getDistance(robot.leftMotor));
            //telemetry.addData("RightMotorDistance ", robot.getDistance(robot.rightMotor));
            //telemetry.addData("Average Distance ", robot.getDistanceStraight());
            //telemetry.update();
            double startTime = System.nanoTime();
            telemetry.addLine("?");
            telemetry.update();
            robot.pidTest(0,0,startTime);
        }
    }
}
