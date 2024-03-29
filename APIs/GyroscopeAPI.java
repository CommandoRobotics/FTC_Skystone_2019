package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class GyroscopeAPI {
    private BNO055IMU imu;
    private float xAngle, yAngle, zAngle;
    float xAngleReset;
    float yAngleReset;
    float zAngleReset;
    

    public GyroscopeAPI(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode    = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        
        this.xAngle = this.yAngle = this.zAngle = 0;
    }

    public void update() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        
        float startX = this.xAngle, startY = this.yAngle, startZ = this.zAngle;
        
        this.xAngle = getRelativeClosestAngle(this.xAngle, orientation.firstAngle);
        this.yAngle = getRelativeClosestAngle(this.yAngle, orientation.secondAngle);
        this.zAngle = getRelativeClosestAngle(this.zAngle, orientation.thirdAngle);
    }
    
    private float getRelativeClosestAngle(float currentAngle, float targetAngle) {
        while(targetAngle - currentAngle >= 360) {
            targetAngle -= 360;
        }
        
        while(targetAngle - currentAngle <= -360) {
            targetAngle += 360;
        }
        
        if(targetAngle != currentAngle) {
            float negativeDirectionTargetAngle = 0;
            float positiveDirectionTargetAngle = 0;
            
            if(targetAngle < currentAngle) {
                negativeDirectionTargetAngle = targetAngle;
                positiveDirectionTargetAngle = targetAngle + 360;
            } else {
                negativeDirectionTargetAngle = targetAngle - 360;
                positiveDirectionTargetAngle = targetAngle;
            }
            float negativeDirectionDistance = negativeDirectionTargetAngle - currentAngle;
            float positiveDirectionDistance = positiveDirectionTargetAngle - currentAngle;
            
            if(Math.abs(positiveDirectionDistance) < Math.abs(negativeDirectionDistance)) return positiveDirectionTargetAngle;
            else return negativeDirectionTargetAngle;
        }
        
        return targetAngle;
    }
    
    public void resetX(){
        xAngleReset = xAngle;
    }
    
    public void resetY(){
        yAngleReset = yAngle;
    }
    
    public void resetZ(){
        zAngleReset = zAngle;
    }
    
    public float getX() {
        return xAngle-xAngleReset;
    }
    
    public float getY() {
        return yAngle-yAngleReset;
    }
    
    public float getZ() {
        return zAngle-zAngleReset;
    }
}