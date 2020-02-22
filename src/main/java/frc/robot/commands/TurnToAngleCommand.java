package frc.robot.commands;

import static frc.robot.Constants.AimConstants.AIM_TOLERANCE;
import static frc.robot.Constants.AimConstants.AIM_VELOCITY_TOLERANCE;
import static frc.robot.Constants.AimConstants.kD;
import static frc.robot.Constants.AimConstants.kP;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * TurnToAngleCommand
 */
public class TurnToAngleCommand extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final PIDController pidController = new PIDController(kP, 0.0, kD);

  public TurnToAngleCommand(double angle, DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.pidController.setTolerance(AIM_TOLERANCE, AIM_VELOCITY_TOLERANCE);
    pidController.setSetpoint(angle);

    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double rotation = pidController.calculate(driveTrainSubsystem.getHeading());
    driveTrainSubsystem.arcadeDrive(0, rotation, false);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0, 0, false);
  }
  
}