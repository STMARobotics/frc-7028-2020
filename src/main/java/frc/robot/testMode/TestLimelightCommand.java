package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.LimelightSubsystem;

public class TestLimelightCommand extends TestCommand {

  private final LimelightSubsystem limelight;
  
  private final NetworkTableEntry networkTableEntry;

  public TestLimelightCommand(LimelightSubsystem limelight) {
    super();

    this.limelight = limelight;

    addRequirements(limelight);
    
    networkTableEntry = getTestModeEntry(limelight.limelightConfig.getNetworkTableName() + "_TargetAcquired");
  }

  @Override
  public void initialize() {
    limelight.enable();
  }

  @Override
  public void execute() {
    networkTableEntry.setBoolean(limelight.getTargetAcquired());
  }

  @Override
  public boolean isFinished() {
    return limelight.getTargetAcquired();
  }

  @Override
  public void end(boolean interrupted) {
    limelight.disable();
  }
}