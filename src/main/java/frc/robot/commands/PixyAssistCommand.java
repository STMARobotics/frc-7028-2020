package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.PixyVisionVariables;

public class PixyAssistCommand extends CommandBase{

  private final PixyVisionSubsystem pixy;
  private final DriveTrainSubsystem driveTrainSubsystem;

  private PixyVisionVariables pixyData;
  private static final int lower = 107; 
  private static final int upper = 147;
  private static final double speed = 0.6;

  public PixyAssistCommand(DriveTrainSubsystem driveTrainSubsystem, PixyVisionSubsystem pixy) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.pixy = pixy;
    addRequirements(driveTrainSubsystem, pixy);
  }

  /** Safe Zone Stuff
   * Possible save zones
   * +/- 27 = 100 to 154
   * +/- 30 = 97 to 157
   * +/- 35 = 92 to 162
   * +/- 40 = 87 to 167
   */


  @Override
  public void execute(){ //go to safezone and follow ball
    pixyData = pixy.getCoordinates(); // {X,Y,Width}
    SmartDashboard.putNumber("X-Coordinate", pixyData.xCoord);
    if((lower<=pixyData.xCoord)&&(pixyData.xCoord<=upper)){
      //Within Safe Zone
      driveTrainSubsystem.tankDrive(speed, speed, false);//drive straight towards the opject in view
      pixyData = pixy.getCoordinates();
      if(pixyData.yCoord>105){ //start intaking

      }
      SmartDashboard.putString("Pixy Command", "Straight");
      SmartDashboard.putNumber("Object Width", pixyData.width);
      SmartDashboard.putNumber("Object Height", pixyData.height);
      SmartDashboard.putNumber("Object Area", pixyData.area);
    }else if(lower>pixyData.xCoord){
      //object is to the left of the bot
      driveTrainSubsystem.tankDrive((speed/5), speed, false);//turn left until object is within the safe zone
      pixyData = pixy.getCoordinates();
      SmartDashboard.putString("Pixy Command", "Left");
      SmartDashboard.putNumber("Object Area", pixyData.  area);
    }else if(pixyData.xCoord>upper){
      //object is to the right of the bot
      driveTrainSubsystem.tankDrive(speed, (speed/5), false);//turn right until object is within the safe zone
      pixyData = pixy.getCoordinates();
      SmartDashboard.putString("Pixy Command", "Right");
      SmartDashboard.putNumber("Object Area", pixyData.area);
    }
  }

  @Override
  public boolean isFinished() {
    return (pixyData.yCoord>113); //start intaking
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.tankDrive(0, 0, false);
  }

}