package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the intake until the command is interrupted.
 */
public class RunIntakeCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;

  public RunIntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }
  
}