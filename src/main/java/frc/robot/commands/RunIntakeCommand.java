package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the intake until the command is interrupted.
 */
public class RunIntakeCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final Supplier<Boolean> indexerIsFull;

  public RunIntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> indexerIsFull) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerIsFull = indexerIsFull;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.intake();
  }

  @Override
  public boolean isFinished() {
    return indexerIsFull.get();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }
  
}