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
        
        double xAxisMultipliedInput = 2*xAxisDistance;
        double yAxisMultipliedInput = 2*yAxisDistance;
        
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
        
        // leftMotorPower = yAxisMultipliedInput;
        // rightMotorPower = yAxisMultipliedInput;
        // strafeMotorPower = -xAxisMultipliedInput;
        
        double radiusOfCircle = Math.sqrt(Math.pow(xAxisMultipliedInput, 2)+Math.pow(yAxisMultipliedInput, 2));
        
        double xAxisControllerInput = Math.toDegrees(radiusOfCircle*Math.cos(Math.toRadians(currentHeading+90)));
        double yAxisControllerInput = Math.toDegrees(radiusOfCircle*Math.sin(Math.toRadians(currentHeading+90)));
        
        leftMotorPower = yAxisControllerInput;
        rightMotorPower = yAxisControllerInput;
        strafeMotorPower = xAxisControllerInput;
        
        if(Math.abs(leftMotorPower) < minimumWheelSpeed){
            leftMotorPower = 0;
        }
        
        if(Math.abs(rightMotorPower) < minimumWheelSpeed){
            rightMotorPower = 0;
        }
        
        if(Math.abs(strafeMotorPower) < minimumWheelSpeed){
            strafeMotorPower = 0;
        }
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