package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ILimelightSubsystem extends Subsystem {
  double getTargetX();

  double getTargetY();

  long getTargetLastSeen();

  void enable();

  void disable();
}
