package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyVisionSubsystem;

public class PIDPixyAssistCommand extends CommandBase {

  private final PixyVisionSubsystem pixy;
  private final DriveTrainSubsystem driveTrainSubsystem;

  private final PIDController xPidController = new PIDController(.005, 0, 0);
  private final PIDController yPidController = new PIDController(.004, 0, 0);

  private boolean hasTargetData = false;

  public PIDPixyAssistCommand(DriveTrainSubsystem driveTrainSubsystem, PixyVisionSubsystem pixy) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.pixy = pixy;
    addRequirements(driveTrainSubsystem, pixy);
    
    // X range is 102-147
    xPidController.setSetpoint(124.5);
    xPidController.setTolerance(22.5);

    //Y range is 115-256
    yPidController.setSetpoint(185.5);
    yPidController.setTolerance(70.5);
  }

  @Override
  public void initialize() {
    xPidController.reset();
    yPidController.reset();
  }

  @Override
  public void execute() {
    var pixyData = pixy.getCoordinates();
    hasTargetData = pixyData.success;
    if (hasTargetData) {
      var speed = yPidController.calculate(pixyData.yCoord);
      var rotation = -xPidController.calculate(pixyData.xCoord);
      driveTrainSubsystem.arcadeDrive(speed, rotation, false);
    }
  }

  @Override
  public boolean isFinished() {
    return !hasTargetData || pixy.getCoordinates().xCoord == 0 ||
        (xPidController.atSetpoint() && yPidController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

}