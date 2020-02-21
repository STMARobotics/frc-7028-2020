package frc.robot.subsystems;

import frc.robot.networktables.DoubleEntryValue;

public interface ILimelightSubsystem {
  DoubleEntryValue getRawTargetValid();

  double getTargetX();
  double getFilteredX();

  double getTargetY();
  double getFilteredY();

  double getDistanceToTarget();

  long getTargetLastSeen();

  void enable();

  void disable();
}
