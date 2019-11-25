/*
Copyright 2018 FIRST Tech Challenge Team 9898a
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
//package org.firstinspires.ftc.roverruckus.teamcode.autonomous;

// import org.firstinspires.ftc.roverruckus.teamcode.apis.*;
// import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.APIs.*;

@Autonomous

public class SkystoneAutonomousMain extends LinearOpMode {

    //private static final int SAMPLE_TIME_ADDITION = 4000;
    //private static final int SAMPLE_START_DELAY = 1000;

    @Override
    public void runOpMode() {

      FTCOmniDriveAPI chassis = new FTCOmniDriveAPI(hardwareMap, telemetry);
      chassis.resetEncoders();
      waitForStart();

      chassis.driveStraightEnc(24,0.5f);
      chassis.driveStrafeEnc(24,.4f);

    }
}
