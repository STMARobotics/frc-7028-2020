package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyVision{
  private int arduinoAddress = 0x14;
  I2C port;

  public PixyVision(){
    port = new I2C(I2C.Port.kOnboard, arduinoAddress);
  }

  public PixyVisionExtension getCoordinates(){
    byte[] xy = new byte[4];
    //if no coordinates are recieved, this will substitute to minimize errors
    PixyVisionExtension revertTo = new PixyVisionExtension(127, 80, 0, false);
    try {
      if(!(port.read(arduinoAddress, 4, xy))){
        //transforms bytes(-128 to 127) into positive integers of desired ranges
        PixyVisionExtension coords = new PixyVisionExtension(xy[0]+128, xy[1]+128, (xy[2]+128)*(xy[3]+128), true);
        SmartDashboard.putString("I2C Failure", "No Failure");
        return coords;
      } else {
        SmartDashboard.putString("I2C Failure", "Failure");
        return revertTo;
      }
    } catch(Exception e) {
      System.out.println("Unexpected exception trying to read data from pixy " + e.getMessage());
      SmartDashboard.putString("I2C Failure", "Failure");
      return revertTo;
    }
  }
}