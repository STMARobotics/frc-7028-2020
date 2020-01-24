package frc.robot.subsystems;

/**
 * PixyVisionExtension
 */
public class PixyVisionExtension {

  public int xCoord;
  public int yCoord;
  public int area;
  public boolean success;

  public PixyVisionExtension(int xCoord, int yCoord, int area, boolean success){
    this.xCoord = xCoord;
    this.yCoord = yCoord;
    this.area = area;
    this.success = success;
  }
}