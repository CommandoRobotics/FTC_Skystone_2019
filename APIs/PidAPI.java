package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PidAPI{
    //set the different PID Modes
    int currentMode;
    public static final int P_MODE = 0;
    public static final int PI_MODE = 1;
    public static final int    PD_MODE = 2;
    public static final int PID_MODE = 3;
    
    double bias, pGain, iGain, dGain, timeConstant, previousError, previousDuration, previousTime;
    
    public PidAPI(int mode, double bias, double pGain, double iGain, double dGain, double timeConstant) {
        //Sets specfic PID mode
        currentMode = mode;
        
        //Sets constants needed for PID formulas
        this.bias = bias; //Value if any PID value = 0, basically what power we are modifying.
        this.pGain = pGain;
        this.iGain = iGain;
        this.dGain = dGain;
        this.timeConstant = timeConstant;
        this.previousError = 0;
        this.previousTime = 0;
    }
    
    //Returns the P/PI/PD/PID output based on mode set and values given and returns bias if all else fails
    public double getOutput(double currentValue, double targetValue, double dt) {
        if(currentMode == P_MODE) return getPOutput(currentValue, targetValue);
        else if(currentMode == PI_MODE) return getPIOutput(currentValue, targetValue, dt);
        else if(currentMode == PD_MODE) return getPDOutput(currentValue, targetValue, dt);
        else if(currentMode == PID_MODE) return getPIDOutput(currentValue, targetValue, dt);
        
        return bias;
    }
    
    //Uses current and target values and just uses P to change bias
    public double getPOutput(double currentValue, double targetValue) {
        double error = currentValue - targetValue;
        return bias + (pGain*error);
    }
    
    //Uses current and target values and dt (change in time) and uses P and I to change bias
    public double getPIOutput(double currentValue, double targetValue, double dt) {
        double error = currentValue - targetValue;
        double errorDuration; 
        
        if ((previousError > 0 && error > 0) || (previousError < 0 || error < 0)) {
            errorDuration = previousDuration + dt;
        } else {
            errorDuration = dt;
        }
        
        previousDuration = errorDuration;
        previousError = error;
        
        return bias + ((pGain*error) + (iGain*error*errorDuration));
    }
    
    //Uses current and target values and dt (change in time) and uses P and D to change bias
    public double getPDOutput(double currentValue, double targetValue, double dt) {
        double error = currentValue - targetValue;
        double changeInError = error - previousError;
        
        previousError = error;
        
        return bias + ((pGain*error) + (dGain*(changeInError/dt)));
    }
    
    //Uses current and target values and dt (change in time) and uses all P, I, and D to change bias
    public double getPIDOutput(double currentValue, double targetValue, double dt) {
        double error = currentValue - targetValue;
        double changeInError = error - previousError;
        double errorDuration; 
        
        if ((previousError > 0 && error > 0) || (previousError < 0 || error < 0)) {
            errorDuration = previousDuration + dt;
        } else {
            errorDuration = dt;
        }
        
        previousDuration = errorDuration;
        previousError = error;
        
        return bias + ((pGain*error) + (iGain*error*errorDuration) + (dGain*(changeInError/dt)));
    }
    
    //Overrides and sets mode to P, PI , PD, or PID mode 
    public void setMode(int mode) {
        this.currentMode = mode;
    }
    
    //Overrides and sets Bias 
    public void setBias(double bias) {
        this.bias = bias;
    }
    
    //Overrides and sets TimeConstant (currently unused)
    public void setTimeConstant(double timeConstant) {
        this.timeConstant = timeConstant;
    }

    //Makes all controller gains positive. Useful if 2 motors are in opposite directions
    public void makeAllGainsPositive() {
        pGain = Math.abs(pGain);
        iGain = Math.abs(iGain);
        dGain = Math.abs(dGain);
    }

    //Makes all controller gains negative. Useful if 2 motors are in opposite directions    
    public void makeAllGainsNegative() {
        pGain = -Math.abs(pGain);
        iGain = -Math.abs(iGain);
        dGain = -Math.abs(dGain);
    }
}
