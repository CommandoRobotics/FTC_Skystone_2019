// /*

// This program was developed on Grover and is not meant for other robotics, but rather as a reference.*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Disabled

public class NewRobotTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        FTCOmniDriveAPI robot = new FTCOmniDriveAPI(hardwareMap);
        telemetry.addLine("Robot initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.driveOmniJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Left Wheel Output: ", robot.leftMotorSpeed);
            telemetry.addData("Right Wheel Output: ", robot.rightMotorSpeed);
            telemetry.addData("Strafe Wheel Output: ", robot.strafeMotorSpeed);
            telemetry.addData("Joystick 1 X Output: ", gamepad1.left_stick_x);
            telemetry.addData("Joystick 1 -Y Output: ", -gamepad1.left_stick_y);
            telemetry.addData("Joystick 2 X Output: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
