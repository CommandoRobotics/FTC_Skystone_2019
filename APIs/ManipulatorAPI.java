public class ManipulatorAPI {
  DcMotor leftManipulatorMotor;
  DcMotor rightManipulatorMotor;
  TouchSensor manipulatorTouchSensor;

  public ManipulatorAPI(HardwareMap hwMap){
    leftManipulatorMotor = hwMap.get(DcMotor.class, "leftManipulatorMotor");
    rightManipulatorMotor = hwMap.get(DcMotor.class, "rightManipulatorMotor");
    manipulatorTouchSensor = hwMap.get(TouchSensor.class, "manipulatorTouchSensor");
  }

  public getIntakeSpeed(double controllerInput){
    if(manipulatorTouchSensor.isPressed()){
      leftManipulatorMotor.setPower(0);
      rightManipulatorMotor.setPower(0);
    } else {
      leftManipulatorMotor.setPower(controllerInput);
      rightManipulatorMotor.setPower(controllerInput);
    }
  }

}
