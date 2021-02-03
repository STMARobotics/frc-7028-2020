package frc.robot.testMode;

import frc.robot.subsystems.ShooterSubsystem;

public class TestShooterCommand extends TestCommand {
  private final ShooterSubsystem shooter;
  private final int targetVelocity;
  private final boolean turnOffShooter;

  private boolean targetVelocityAcquired = false;

  public TestShooterCommand(ShooterSubsystem shooter, int targetVelocity) {
    this(shooter, targetVelocity, false);
  }

  public TestShooterCommand(ShooterSubsystem shooter, int targetVelocity, boolean turnOffShooter) {
    super();

    this.shooter = shooter;
    this.targetVelocity = targetVelocity;
    this.turnOffShooter = turnOffShooter;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
     super.execute();

    shooter.setRawVelocity(targetVelocity);
  }

  @Override
  public boolean isFinished() {
    //finish if we're within 25

    targetVelocityAcquired = Math.abs(shooter.getVelocity() - targetVelocity) < 25;

    return targetVelocityAcquired;
  }

  @Override
  public void end(boolean interrupted) {
    var testModeName = "TestShooterCommand_" + targetVelocity;
    getTestModeEntry(testModeName).setBoolean(targetVelocityAcquired);
    getTestModeEntry(testModeName + "_ActionVelocity").setNumber(shooter.getVelocity());

    if(turnOffShooter) {
      shooter.setRawVelocity(0);
    }
  }
}