import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorAPI {
  
  ColorSensor color;
  double[3] getRGBValue;
  
  
  public ColorSensorAPI() {
    color = new ColorSensor();
    color.enableLed(true);

  }
  
  public double[] getRGBValue() {
    this.getRGBValue[0] = colorsensor.int red();
    this.getRGBValue[1] = colorsensor.int green();
    this.getRGBValue[2] = colorsensor.int blue();
    this.system.out.println(getRGBValue[0], getRGBValue[1], getRGBValue[2]);
  }
  
  public void int red() {
    return this.colorSensor.red();
  }
  
  public void int green() {
    return this.colorSensor.green();
  }
  
  public void int blue() {
    return this.colorSensor.blue();
  }
  
  public void int isYellow() {
    
    
  }
  public void int isBlack() {
    
    
  }
}