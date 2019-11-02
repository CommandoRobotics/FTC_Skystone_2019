
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorAPI {
  
  ColorSensor color;
  void enableLed(true);
  
  public ColorSensorAPI() {
     color = new ColorSensor(#);
  }
  
  public double[] getRGBValue() {
    getRGBValue = new double[3];
    getRGBValue[0] = colorsensor.int red();
    getRGBValue[1] = colorsensor.int green ();
    getRGBValue[2] = colorsensor.int blue ();
  }
  
  public void int red() {
    return this.colorSensor.red();
  }
  
  public void int green() {
    return this.colorSensor.green();
  }
  
  public void int blue() {
    return this.colorSensor.bLue();
  }
  
  public void int isYellow(){
    
    
  }
  public void int isBlack(){
    
    
  }

  
  system.out.println(getRGBValue[0,1,2])
}