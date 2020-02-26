package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * Moves the climb up and down
 */
public class TestClimb extends TestCommand {
  
  private static final int testRotations = 3;

  private final ClimbSubsystem climbSubsystem;
  private final NetworkTableEntry networkTableEntry = getTestModeEntry("ClimbTest");

  private double startPosition;
  private boolean raised;

  public TestClimb(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    networkTableEntry.setBoolean(false);
    startPosition = climbSubsystem.getClimbPosition();
    raised = false;
  }

  @Override
  public void execute() {
    if (raised) {
      climbSubsystem.lowerClimb();
      if (climbSubsystem.getClimbPosition() <= startPosition) {
        climbSubsystem.stopClimb();
        networkTableEntry.setBoolean(true);
      }
    } else {
      climbSubsystem.raiseClimb();
      if (climbSubsystem.getClimbPosition() >= (startPosition + testRotations)) {
        climbSubsystem.stopClimb();
        raised = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopClimb();
  }
  
}