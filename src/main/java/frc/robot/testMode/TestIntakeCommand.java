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

  private int encoderStart = 0;
  private int lastPosition = 0;

  public TestIntakeCommand(boolean forward, IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.forward = forward;

    addRequirements(intakeSubsystem);
    
    var direction = forward ? "Forward" : "Reverse";
    networkTableEntry = getTestModeEntry("IntakeTest_" + direction);
  }

  @Override
  public void initialize() {
    super.initialize();

    encoderStart = intakeSubsystem.getEncoderPosition();
    lastPosition = encoderStart;
  }

  @Override
  public void execute() {
    super.execute();
    
    var currentPosition = intakeSubsystem.getEncoderPosition();

    if (forward) {
      intakeSubsystem.intake();
      networkTableEntry.setBoolean(currentPosition > lastPosition);
    } else {
      intakeSubsystem.reverse();
      networkTableEntry.setBoolean(currentPosition < lastPosition);
    }

    lastPosition = currentPosition;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (forward) {
      networkTableEntry.setBoolean(lastPosition - encoderStart > 1000);
    } else {
      networkTableEntry.setBoolean(encoderStart - lastPosition > 1000);
    }
    intakeSubsystem.stopIntake();
  }

  
}