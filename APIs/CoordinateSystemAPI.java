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

        double goldenRatio;

        boolean xIsGreater = false;
        boolean yIsGreater = false;

        double radiusOfCircle = Math.sqrt(Math.pow(xAxisMultipliedInput, 2)+Math.pow(yAxisMultipliedInput, 2));

        double xAxisControllerInput = Math.toDegrees(radiusOfCircle*Math.cos(Math.toRadians(currentHeading+180)));
        double yAxisControllerInput = Math.toDegrees(radiusOfCircle*Math.sin(Math.toRadians(currentHeading+180)));

        if(Math.abs(xAxisControllerInput) > Math.abs(yAxisControllerInput)){
            goldenRatio = yAxisControllerInput/xAxisControllerInput;
            xIsGreater = true;
        } else if(Math.abs(yAxisControllerInput) > Math.abs(xAxisControllerInput)){
            goldenRatio = xAxisControllerInput/yAxisControllerInput;
            yIsGreater = true;
        } else {
            goldenRatio = 1;
            xIsGreater = false;
            yIsGreater = false;
        }

        if(xIsGreater){
            if(xAxisControllerInput > 1){
                xAxisControllerInput = 1;
                if(yAxisControllerInput > 0){
                    yAxisControllerInput = goldenRatio*1;
                } else if(xAxisControllerInput < 0){
                    yAxisControllerInput = goldenRatio*-1;
                }
            } else if(xAxisControllerInput < -1){
                xAxisControllerInput = -1;
                if(yAxisControllerInput > 0){
                    yAxisControllerInput = goldenRatio*1;
                } else if(xAxisControllerInput < 0){
                    yAxisControllerInput = goldenRatio*-1;
                }
            }
        } else if(yIsGreater){
            if(yAxisControllerInput > 1){
                yAxisControllerInput = 1;
                if(xAxisControllerInput > 0){
                    xAxisControllerInput = goldenRatio*1;
                } else if(yAxisControllerInput < 0){
                    xAxisControllerInput = goldenRatio*-1;
                }
            } else if(yAxisControllerInput < -1){
                yAxisControllerInput = -1;
                if(xAxisControllerInput > 0){
                    xAxisControllerInput = goldenRatio*1;
                } else if(yAxisControllerInput < 0){
                    xAxisControllerInput = goldenRatio*-1;
                }
            }
        } else {
            if(xAxisControllerInput > 1){
                xAxisControllerInput = 1;
                yAxisControllerInput = 1;
            } else if(xAxisControllerInput < -1){
                xAxisControllerInput = -1;
                yAxisControllerInput = -1;
            }
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
