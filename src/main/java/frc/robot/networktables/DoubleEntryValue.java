package frc.robot.networktables;

import java.time.LocalDateTime;

public class DoubleEntryValue {

  public DoubleEntryValue(double value) {
    super();

    Value = value;
    UpdateTime = LocalDateTime.now();
  }
  
  public double Value;
  public LocalDateTime UpdateTime;
}