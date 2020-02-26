package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * IntakeTest
 */
public class TestIntakeCommand extends TestCommand {

  private final boolean forward;
  private final IntakeSubsystem intakeSubsystem;
  private final NetworkTableEntry networkTableEntry;

  public TestIntakeCommand(boolean forward, IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.forward = forward;

    addRequirements(intakeSubsystem);
    
    var direction = forward ? "Forward" : "Reverse";
    networkTableEntry = getTestModeEntry("IntakeTest_" + direction);
  }

  @Override
  public void execute() {
    super.execute();    
    if (forward) {
      intakeSubsystem.intake();
    } else {
      intakeSubsystem.reverse();
    }

  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (forward) {
      networkTableEntry.setBoolean(true);
    } else {
      networkTableEntry.setBoolean(true);
    }
    intakeSubsystem.stopIntake();
  }
   
}