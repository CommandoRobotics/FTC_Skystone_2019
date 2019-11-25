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
        double xAxisDistance = targetX-currentX;
        double yAxisDistance = targetY-currentY;

        double xAxisMultipliedInput = xAxisDistance;
        double yAxisMultipliedInput = yAxisDistance;

        if(xAxisMultipliedInput > 100){
            xAxisMultipliedInput = 1;
        } else if(xAxisMultipliedInput < -100){
            xAxisMultipliedInput = -1;
        } else {
            xAxisMultipliedInput = 0.01*xAxisMultipliedInput;
        }

        if(yAxisMultipliedInput > 100){
            yAxisMultipliedInput = 1;
        } else if(yAxisMultipliedInput < -100){
            yAxisMultipliedInput = -1;
        } else {
            yAxisMultipliedInput = 0.01*yAxisMultipliedInput;
        }

        leftMotorPower = yAxisMultipliedInput;
        rightMotorPower = yAxisMultipliedInput;
        strafeMotorPower = -xAxisMultipliedInput;

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
