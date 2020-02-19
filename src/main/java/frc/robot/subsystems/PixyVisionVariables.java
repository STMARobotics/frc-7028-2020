package frc.robot.subsystems;

/**
 * PixyVisionExtension
 */
public class PixyVisionVariables {

  public int xCoord;
  public int yCoord;
  public int width;
  public boolean success;
  public int area;
  public int height;

  public PixyVisionVariables(int xCoord, int yCoord, int width, int height, int area, boolean success){
    this.xCoord = xCoord;
    this.yCoord = yCoord;
    this.width = width;
    this.success = success;
    this.area = area;
    this.height = height;
  }
}