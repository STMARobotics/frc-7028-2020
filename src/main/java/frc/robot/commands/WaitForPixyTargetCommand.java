package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PixyVisionSubsystem;

/**
 * This command won't finish until getTargetAcquired returns true, it should be used with a timeout
 */
public class WaitForPixyTargetCommand extends CommandBase {

  private final PixyVisionSubsystem pixy;

  public WaitForPixyTargetCommand(PixyVisionSubsystem pixy) {

    this.pixy = pixy;

    addRequirements(pixy);
  }

  @Override
  public boolean isFinished() {
    return pixy.getCoordinates().success;
  }
}