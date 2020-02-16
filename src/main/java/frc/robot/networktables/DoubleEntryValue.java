package frc.robot.networktables;

public class DoubleEntryValue {

  public DoubleEntryValue(double value) {
    super();

    Value = value;
    UpdateTime = System.currentTimeMillis();
  }

  public double Value;
  public long UpdateTime;
}