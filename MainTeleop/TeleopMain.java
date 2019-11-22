package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.teamcode.APIs.FTCOmniDriveAPI;
import org.firstinspires.ftc.teamcode.APIs.CoordinateSystemAPI;

@TeleOp(name="TeleopMain")

public class TeleopMain extends LinearOpMode {

  private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  private static final boolean PHONE_IS_PORTRAIT = true  ;

    /*
    * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
    * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
    * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
    * web site at https://developer.vuforia.com/license-manager.
    *
    * Vuforia license keys are always 380 characters long, and look as if they contain mostly
    * random data. As an example, here is a example of a fragment of a valid key:
    *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
    * Once you've obtained a license key, copy the string from the Vuforia web site
    * and paste it in to your code on the next line, between the double quotes.
    */
    private static final String VUFORIA_KEY =
        "AV94enn/////AAABmZe9biDtTkKftQJ2GBQvyMwXche1ItTA6lbJ/G08UOMB3OlV1SrrFIMmenj7C2jBLZGrcJ9qdwy4lzk8MWh6h5nmQD2f1FJudNEvCac2WG5DCVr8eB+Is+5XlQugWTvADGMM4dL23BP3vxVkfVms6yC7QiDb7FJSRjjKGiFBNiIgVvK8XsonVoml65VO8+A4OW1KmV9T29A2+w5TN2SpdxtKk+6/h4BDZehziYDLfyyWVU69BQT9FkrGGMSTf5420NiUG8cym/F3vqyWyWK4+sdv1o+asqrktLSzSdukJaPd17pK2R8fs5alSlC/108bTwpQuZfeyfs303VsgWKoDvf+3VUUfLYhhjRjoxDHSlgO";

          // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0f;
    private float phoneYRotate    = -90f;
    private float phoneZRotate    = 0f;

    double robotLocationX;
    double robotLocationY;

    @Override
    public void runOpMode() {
        //TouchSensor intakeButton = hardwareMap.get(TouchSensor.class, "intakeButton");
        //ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        telemetry.addLine("Robot Status : Creating Camera");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        telemetry.addLine("Robot Status : Setting Parameters");
        telemetry.update();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;
        telemetry.addLine("Robot Status : Creating Vuforia Object");
        telemetry.update();
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addLine("Robot Status : Loading Trackables");
        telemetry.update();
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        telemetry.addLine("Robot Status : Setting Trackable Names");
        telemetry.update();
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        telemetry.addLine("Robot Status : Gathering All Trackables");
        telemetry.update();
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        telemetry.addLine("Robot Status : Setting Trackable Location");
        telemetry.update();

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        telemetry.addLine("Robot Status : Declaring Some Constants");
        telemetry.update();


        final float CAMERA_FORWARD_DISPLACEMENT  = 1.275f * mmPerInch;   // eg: 4f if Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 11.625f * mmPerInch;   // eg: 8f if Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 7;     // eg: 0f if Camera is ON the robot's center line

        telemetry.addLine("Robot Status : Setting Camera Location");
        telemetry.update();


        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */
        // for (VuforiaTrackable trackable : allTrackables) {
        //     ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        // }

        FTCOmniDriveAPI RIPSteve = new FTCOmniDriveAPI(hardwareMap);
        CoordinateSystemAPI coordinateTest = new CoordinateSystemAPI(0.2);

        gamepad1.setJoystickDeadzone(0);

        telemetry.addLine("Robot Status : Initialized");
        telemetry.update();

        waitForStart();

        targetsSkyStone.activate();






        while (opModeIsActive()) {
            RIPSteve.driveOmniJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            // telemetry.addData("Strafe Wheel inch", RIPSteve.getDistanceStrafe());
            // telemetry.addData("ave forward inch", RIPSteve.getDistanceStraight());
            // telemetry.addData("totalDistance", RIPSteve.getDisPerPulse() * 1120);
            // telemetry.update();






            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                } else {
                  targetVisible = false;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                robotLocationX = translation.get(0)/mmPerInch;
                robotLocationY = translation.get(1)/mmPerInch;
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }

            boolean isAPressed = false;
            boolean activateAutoPilot = false;

            if(gamepad1.a){
              if(isAPressed == false){
                isAPressed = true;
                if(activateAutoPilot){
                  activateAutoPilot = false;
                } else {
                  activateAutoPilot = true;
                }
              }
            } else {
              isAPressed = false;
            }

            coordinateTest.calculateCoordinates(robotLocationX, robotLocationY, 90, 10.0, 0.0, 0.0);
            telemetry.addData("Left Power", coordinateTest.coordinatesLeftMotorPower());
            telemetry.addData("Right Power", coordinateTest.coordinatesRightMotorPower());
            telemetry.addData("Strafe Power", coordinateTest.coordinatesStrafeMotorPower());
            if(activateAutoPilot){
              telemetry.addLine("Auto Pilot Activated");
              RIPSteve.controlChassis(coordinateTest.coordinatesLeftMotorPower(), -coordinateTest.coordinatesRightMotorPower(), coordinateTest.coordinatesStrafeMotorPower());
            }
            telemetry.addData("Robot rotation", RIPSteve.getRotation());
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }
}
