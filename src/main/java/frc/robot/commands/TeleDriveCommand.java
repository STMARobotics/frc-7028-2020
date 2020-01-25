package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * TeleDriveCommand
 */
public class TeleDriveCommand extends CommandBase {

  public static final double ROTATION_MULTIPLIER = .78;

  public static final double SLOW_MODE_SPEED_MULTIPLIER = .6;
  public static final double SLOW_MODE_ROTATION_MULTIPLIER = .9;

  private final XboxController driverController;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private boolean slowMode = false;
  private boolean reverseMode = false;

  public TeleDriveCommand(XboxController driverController, DriveTrainSubsystem driveTrainSubsystem) {
    this.driverController = driverController;
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void execute() {
    double speed = getSpeed();
    if (getReverseMode()) {
      speed = -speed;
    }
    driveTrainSubsystem.arcadeDrive(speed, getRotation(), true);
  }

  private double getSpeed() {
    double speed;
    if (getSlowMode()) {
      speed = driverController.getY(Hand.kLeft) * SLOW_MODE_SPEED_MULTIPLIER;
    } else {
      speed = driverController.getY(Hand.kLeft);
    }
    return -speed;
  }

  private double getRotation() {
    double rotation;
    if (getSlowMode()) {
      rotation = driverController.getX(Hand.kRight) * ROTATION_MULTIPLIER * SLOW_MODE_ROTATION_MULTIPLIER;
    } else {
      rotation = driverController.getX(Hand.kRight) * ROTATION_MULTIPLIER;
    }
    return rotation;
  }

  private boolean getSlowMode() {
    if (driverController.getBButtonPressed()) {
      slowMode = !slowMode;
    }
    return slowMode;
  }

  private boolean getReverseMode() {
    if (driverController.getAButtonPressed()) {
      reverseMode = !reverseMode;
    }
    return reverseMode;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0, 0);
  }

}