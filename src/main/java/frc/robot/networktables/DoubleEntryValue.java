package frc.robot.networktables;

public class DoubleEntryValue {

  /**
   * Raw Value from Network Table
   */
  public final double value;

  /**
   * Time (in ms) of update
   */
  public final long updateTime;

  public DoubleEntryValue(double value) {
    super();

    this.value = value;
    this.updateTime = System.currentTimeMillis();
  }
}