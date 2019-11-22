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

  public boolean isBlack() {
    if (color.red() < 10 && color.blue() < 10 && color.green() < 10) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isYellow() {
    if ((color.red() > 210 && color.red() > 190) && (color.green() > 170 && color.green() < 195) && color.blue() < 50) {
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
