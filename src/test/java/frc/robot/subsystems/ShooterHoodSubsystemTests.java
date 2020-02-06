package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Unit tests for DriveTrainSubSystem
 */
public class ShooterHoodSubsystemTests {
  private final double delta = .001;

  @Test
  public void testPercentageToSpeed_MaxValue() {
    //arrange
    final double expected = 1, min = -1, max = 1;
    
    //act
    var actual = ShooterHoodSubsystem.percentageToSpeed(1, min, max);

    //assert
    assertEquals(expected, actual, delta);

    actual = ShooterHoodSubsystem.percentageToSpeed(2, min, max);

    //max percent is 1, so passing 2 shouldn't change the output
    assertEquals(expected, actual, delta);
  }

  @Test
  public void testPercentageToSpeed_MinValue() {
    //arrange
    final double expected = -1, min = -1, max = 1;

    //act
    var actual = ShooterHoodSubsystem.percentageToSpeed(0, min, max);
    assertEquals(expected, actual, delta);

    //min percent is 0, so passing -1 should't change it
    actual = ShooterHoodSubsystem.percentageToSpeed(-1, min, max);
    assertEquals(-1, actual, delta);
  }

  @Test
  public void testPercentageToSpeed_75PercentRange() {
    //arrange
    final double min = -1, max = .5;

    //act
    //0% test
    var actual = ShooterHoodSubsystem.percentageToSpeed(0, min, max);
    assertEquals(min, actual, delta);

    //20% test
    actual = ShooterHoodSubsystem.percentageToSpeed(.2, min, max);
    assertEquals(-.7, actual, delta);

    //50%
    actual = ShooterHoodSubsystem.percentageToSpeed(.5, min, max);
    assertEquals(-.25, actual, delta);

    //50%
    actual = ShooterHoodSubsystem.percentageToSpeed(1, min, max);
    assertEquals(max, actual, delta);
  }

  @Test
  public void testPercentageToSpeed_Middle50PercentRange() {
    //arrange
    final double min = -.5, max = .5;

    //act
    //0% test
    var actual = ShooterHoodSubsystem.percentageToSpeed(0, min, max);
    assertEquals(min, actual, delta);

    //20% test
    actual = ShooterHoodSubsystem.percentageToSpeed(.2, min, max);
    assertEquals(-.3, actual, delta);

    //50%
    actual = ShooterHoodSubsystem.percentageToSpeed(.5, min, max);
    assertEquals(0, actual, delta);

    //62%
    actual = ShooterHoodSubsystem.percentageToSpeed(.62, min, max);
    assertEquals(.12, actual, delta);

    //50%
    actual = ShooterHoodSubsystem.percentageToSpeed(1, min, max);
    assertEquals(max, actual, delta);
  }
}