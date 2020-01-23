package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;

public class PixyVision{
  private int arduinoAddress = 0x14;
  I2C port;

  public PixyVision(){
    port = new I2C(I2C.Port.kOnboard, arduinoAddress);
  }

  public int[] getCoordinates(){
    byte[] xy = new byte[4];
    //if no coordinates are recieved, this will substitute to minimize errors
    int[] revertTo = {127, 80, 0};
    try {
      if(!(port.read(arduinoAddress, 4, xy))){
        //transforms bytes(-128 to 127) into positive integers of desired ranges
        int[] coords = {xy[0]+128, xy[1]+128, (xy[2]+128)*(xy[3]+128)};
        return coords;
      } else {
        return revertTo;
      }
    } catch(Exception e) {
      System.out.println("Unexpected exception trying to read data from pixy " + e.getMessage());
      return revertTo;
    }
  }
}