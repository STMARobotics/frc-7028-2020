package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.commands.WaitForTargetCommand;
import frc.robot.subsystems.LimelightSubsystem;

public class TestLimelightCommand extends TestCommand
{
  private LimelightSubsystem limelight;
  private final WaitForTargetCommand waitCommand;
  
  private final NetworkTableEntry networkTableEntry;

  public TestLimelightCommand(LimelightSubsystem limelight) {
    super();

    this.limelight = limelight;

    waitCommand = new WaitForTargetCommand(limelight);
    limelight.enable();

    networkTableEntry = getTestModeEntry(limelight.limelightConfig.getNetworkTableName() + "_TargetAcquired");
  }

  @Override
  public void execute() {
    super.execute();

    waitCommand.execute();

    networkTableEntry.setBoolean(waitCommand.isFinished());
  }

  @Override
  public boolean isFinished() {
    return waitCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    limelight.disable();
  }
}