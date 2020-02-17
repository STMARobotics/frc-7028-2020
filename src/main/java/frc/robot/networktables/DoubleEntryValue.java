package frc.robot.networktables;

public class DoubleEntryValue {

  public DoubleEntryValue(double value) {
    this(value, value);
  }

  public DoubleEntryValue(double value, double filteredValue) {
    super();

    Value = value;
    UpdateTime = System.currentTimeMillis();
  }

  /**
   * Raw Value from Network Table
   */
  public double Value;

  /** 
   * Optional Filtered Value
   */
  public double FilteredValue;

  /**
   * Time (in ms) of update
   */
  public long UpdateTime;
}