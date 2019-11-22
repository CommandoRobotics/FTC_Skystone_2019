package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.APIs.FTCOmniDriveAPI;

@TeleOp(name="BishopKelleyQualifier")

public class BishopKelleyQualifier extends LinearOpMode {

    @Override
    public void runOpMode() {

        FTCOmniDriveAPI robot = new FTCOmniDriveAPI(hardwareMap);

        gamepad1.setJoystickDeadzone(0);

        telemetry.addLine("Robot Status : Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.driveOmniJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
