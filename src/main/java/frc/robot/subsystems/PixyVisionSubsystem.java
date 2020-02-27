package frc.robot.subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class PixyVisionSubsystem extends SubsystemBase {
  private final int arduinoAddress = 0x14;
  private final I2C port;
  private final int minimumPollingWait = 5, maximumPollingWait = 20;

  //if no coordinates are recieved, this will substitute to minimize errors
  private final PixyVisionVariables revertTo = new PixyVisionVariables(127, 80, 0, 0, 0, false);

  private int pollingMs;
  private PixyVisionVariables coordinates;
  private boolean polling;

  public PixyVisionSubsystem() {
    port = new I2C(I2C.Port.kOnboard, arduinoAddress);

    pollingMs = 10;

    coordinates = revertTo; //default to the error case until the polling kicks in

    //create a polling thread that handles direct interaction with the Pixy
    var executor = Executors.newCachedThreadPool();
    executor.submit(() -> this.run());
  }

  private void run() {

    if (polling) {
      //copy the current values to a previous variable for comparison
      var prevCoordinates = coordinates;

      //grab the current values from the pixy
      coordinates = getCoordinatesPrivate();

      //not sure how often the pixy will have updates, this should tune down/up our 
      //wait time to check as often as we're seeing updates come through
      
      //if they're the same we increase the wait time, if they changed we decrease
      if (prevCoordinates.xCoord == coordinates.xCoord && prevCoordinates.yCoord == coordinates.yCoord) {
        pollingMs++;
      } else {
        pollingMs--;
      }

      //make sure we stay between our min/max wait times
      pollingMs = MathUtil.clamp(pollingMs, minimumPollingWait, maximumPollingWait);
    }

    //try to sleep set amount of wait time before polling again
    try {

      TimeUnit.MILLISECONDS.sleep(pollingMs);
    } catch (InterruptedException e) {
      
      e.printStackTrace();
    }
  }

  public PixyVisionVariables getCoordinates(){
    polling = true; //start polling every time we call get coordinates
    return coordinates;
  }

  private PixyVisionVariables getCoordinatesPrivate(){
    byte[] xy = new byte[4];
    
    try {
      if(!(port.read(arduinoAddress, 4, xy))){
        //transforms bytes(-128 to 127) into positive integers of desired ranges
        PixyVisionVariables coords = new PixyVisionVariables(xy[0]+128, xy[1]+128, (xy[2]+128), (xy[3]+128), (xy[2]+128)*(xy[3]+128), true);
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

  public void endPolling() {
    polling = false;
    coordinates = revertTo; //revert back to default/error state for next round of polling
  }
}