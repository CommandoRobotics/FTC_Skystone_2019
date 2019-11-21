package org.firstinspires.ftc.teamcode.APIs;

import java.lang.Math;

public class CoordinateSystemAPI {

    double leftMotorPower;
    double rightMotorPower;
    double strafeMotorPower;

    double minimumWheelSpeed;

    public CoordinateSystemAPI(double minSpeed){
        minimumWheelSpeed = minSpeed;
    }

    public void calculateCoordinates(double currentX, double currentY, double currentHeading, double targetX, double targetY, double targetHeading){

        private static final double X_AXIS_SCALE = 6000;
        private static final double Y_AXIS_SCALE = 1000;
        double xAxisDistance = targetX-currentX;
        double yAxisDistance = targetY-currentY;

        double xAxisMultipliedInput = xAxisDistance;
        double yAxisMultipliedInput = yAxisDistance;

        double goldenRatio;
        double newCurrentHeading;

        //Calculate the radius of the circle using the distance formula
        double radiusOfCircle = Math.sqrt(Math.pow(xAxisMultipliedInput-targetX, 2)+Math.pow(yAxisMultipliedInput-targetY, 2));

        //Flip the heading 180 degrees
        if(currentHeading+180 > 360){
            newCurrentHeading = currentHeading-180;
        } else {
            newCurrentHeading = currentHeading+180;
        }

        //Find the x and y value of the target point after it has been moved to the correct angle
        double xAxisControllerInput = radiusOfCircle*(Math.toDegrees(Math.cos(Math.toRadians(newCurrentHeading))));
        double yAxisControllerInput = radiusOfCircle*(Math.toDegrees(Math.sin(Math.toRadians(newCurrentHeading))));

        if(xAxisControllerInput > yAxisControllerInput){
          goldenRatio = yAxisControllerInput/xAxisControllerInput;
          xAxisControllerInput = xAxisControllerInput/X_AXIS_SCALE;
          yAxisControllerInput = yAxisControllerInput*goldenRatio;
        } else if(yAxisControllerInput > xAxisControllerInput){
          goldenRatio = xAxisControllerInput/yAxisControllerInput;
          yAxisControllerInput = yAxisControllerInput/Y_AXIS_SCALE;
          xAxisControllerInput = xAxisControllerInput*goldenRatio;
        }

        leftMotorPower = yAxisControllerInput;
        rightMotorPower = yAxisControllerInput;
        strafeMotorPower = xAxisControllerInput;
    }

    public double coordinatesLeftMotorPower(){
        return leftMotorPower;
    }

    public double coordinatesRightMotorPower(){
        return rightMotorPower;
    }

    public double coordinatesStrafeMotorPower(){
        return strafeMotorPower;
    }

}
