package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyVision;
import frc.robot.subsystems.PixyVisionExtension;

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
  private PixyVisionExtension pixyData = pixy.getCoordinates(); // {X,Y,Area}
  private double speed = 0.4;

  public void goToSafeZone(){
    pixyData = pixy.getCoordinates(); // {X,Y,Area}
    SmartDashboard.putNumber("Coordinate", pixyData.xCoord);
    if((lower<=pixyData.xCoord)&&(pixyData.xCoord<=upper)){
      //Within Safe Zone
      driveTrainSubsystem.tankDrive(speed, speed, false);//drive straight towards the opject in view
      pixyData = pixy.getCoordinates();
      
      
      SmartDashboard.putString("Pixy Command", "Straight");
    }else if(lower>pixyData.xCoord){
      //object is to the left of the bot
      driveTrainSubsystem.tankDrive((speed/4), speed, false);//turn left until object is within the safe zone
      pixyData = pixy.getCoordinates();
      SmartDashboard.putString("Pixy Command", "Left");
    }else if(pixyData.xCoord>upper){
      //object is to the right of the bot
      driveTrainSubsystem.tankDrive(speed, (speed/4), false);//turn right until object is within the safe zone
      pixyData = pixy.getCoordinates();
      SmartDashboard.putString("Pixy Command", "Right");
    }
  }
}