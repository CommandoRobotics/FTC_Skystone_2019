package org.firstinspires.ftc.teamcode.APIs;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CapstonePlacerAPI {

  Servo capstonePlacer;

  public CapstonePlacerAPI(HardwareMap hardwareMap){
    capstonePlacer = hardwareMap.get(Servo.class, "capstonePlacer");
  }

  public void place(){
    capstonePlacer.setPosition(90);
    long startTime = System.currentTimeMillis();
    while(System.currentTimeMillis()-startTime < 1000){

    }
    capstonePlacer.setPosition(0);
  }

}
