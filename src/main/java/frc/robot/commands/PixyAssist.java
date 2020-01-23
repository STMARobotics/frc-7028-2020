package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyVision;

public class PixyAssist {

  private final PixyVision pixy = new PixyVision();
  private final DriveTrainSubsystem driveTrainSubsystem;

  public PixyAssist(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
  }

  /** Safe Zone Stuff
   * Possible save zones
   * +/- 27 = 100 to 154
   * +/- 30 = 97 to 157
   * +/- 35 = 92 to 162
   * +/- 40 = 87 to 167
   */

  private int lower = 100; 
  private int upper = 154;
  private int[] coords = pixy.getCoordinates(); // {X,Y}
  private double speed = 0.4;

  public void goToSafeZone(){
    coords = pixy.getCoordinates(); // {X,Y}
    SmartDashboard.putNumber("Coordinate", coords[0]);
    if((lower<=coords[0])&&(coords[0]<=upper)){
      //Within Safe Zone
      driveTrainSubsystem.tankDrive(speed, speed, false);//drive straight towards the opject in view
      coords = pixy.getCoordinates();
      SmartDashboard.putString("Pixy Command", "Straight");
    }else if(lower>coords[0]){
      //object is to the left of the bot
      driveTrainSubsystem.tankDrive((speed/4), speed, false);//turn left until object is within the safe zone
      coords = pixy.getCoordinates();
      SmartDashboard.putString("Pixy Command", "Left");
    }else if(coords[0]>upper){
      //object is to the right of the bot
      driveTrainSubsystem.tankDrive(speed, (speed/4), false);//turn right until object is within the safe zone
      coords = pixy.getCoordinates();
      SmartDashboard.putString("Pixy Command", "Right");
    }
  }
}