package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.PixyVisionVariables;

public class PixyAssistCommand extends CommandBase{

  private final PixyVisionSubsystem pixy;
  private final DriveTrainSubsystem driveTrainSubsystem;

  private static final int lower = 100; 
  private static final int upper = 154;
  private static final double speed = 0.4;

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
    PixyVisionVariables pixyData = pixy.getCoordinates(); // {X,Y,Area}
    SmartDashboard.putNumber("X-Coordinate", pixyData.xCoord);
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

  @Override
  public boolean isFinished() {
    return false;
  }

}