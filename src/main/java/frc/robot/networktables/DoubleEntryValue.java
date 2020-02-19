package frc.robot.networktables;

public class DoubleEntryValue {

  /**
   * Raw Value from Network Table
   */
  public double value;

  /** 
   * Optional Filtered Value
   */
  public double filteredValue;

  /**
   * Time (in ms) of update
   */
  public long updateTime;

  public DoubleEntryValue(double value) {
    this(value, value);
  }

  public DoubleEntryValue(double value, double filteredValue) {
    super();

    this.value = value;
    this.updateTime = System.currentTimeMillis();
  }
}