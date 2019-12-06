package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorAPI {

  ColorSensor color;

  public ColorSensorAPI(HardwareMap hwMap, String sensorName) {
    color = hwMap.get(ColorSensor.class, sensorName);
  }

    public double getRed() {
        return color.red();
    }
    
    public double getGreen() {
        return color.green();
    }
    
    public double getBlue() {
        return color.blue();
    }

  public boolean isBlack() {
    if ((color.red() > 140 && color.red() < 155) && (color.green() < 260 && color.green() > 230)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isYellow() {
    if ((color.red() > 200 && color.red() < 250) && (color.green() > 300 && color.green() < 370)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isSkystoneDetected() {
    if (isBlack() && !isYellow()) {
      return true;
    } else {
      return false;
    }
  }
}
