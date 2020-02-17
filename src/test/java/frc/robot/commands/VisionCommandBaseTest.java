package frc.robot.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.when;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnitRunner;

import frc.robot.Constants;
import frc.robot.networktables.DoubleEntryValue;
import frc.robot.subsystems.ILimelightSubsystem;

@RunWith(MockitoJUnitRunner.class)
public class VisionCommandBaseTest {

  @Mock
  private ILimelightSubsystem limelightHigh;

  @Mock
  private ILimelightSubsystem limelightLow;

  private VisionCommandBase getCommand() {
    return new VisionCommandBase(limelightHigh, limelightLow);
  }

  @Test
  public void testNullTargetValidValue() {
    //arrange
    when(limelightHigh.getRawTargetValid()).thenReturn(null);
    when(limelightLow.getRawTargetValid()).thenReturn(null);
    
    var command = getCommand();

    //act
    var actual = command.getTargetAcquired();

    //assert
    assertFalse("Target shouldn't be acquired with null values", actual);
  }

  @Test
  public void testOneTargetValidValue() {
    //arrange
    when(limelightHigh.getRawTargetValid()).thenReturn(null);
    when(limelightLow.getRawTargetValid()).thenReturn(new DoubleEntryValue(Constants.LimeLightConstants.TARGET_ACQUIRED));
    
    var command = getCommand();

    //act
    var actual = command.getTargetAcquired();

    //assert
    assertTrue("Target should be acquired with one value", actual);
  }

  @Test
  public void testTargetValidValueFalseDelayNotPassed() {
    //arrange
    when(limelightHigh.getRawTargetValid()).thenReturn(null);
    when(limelightLow.getRawTargetValid()).thenReturn(new DoubleEntryValue(0.0));
    
    var command = getCommand();

    //act
    var actual = command.getTargetAcquired();

    //assert
    assertTrue("Target should be acquired with 500 ms delay in effect", actual);
  }

  @Test
  public void testTargetValidValueFalseDelayPassed() {
    //arrange
    var passedValue = new DoubleEntryValue(0.0);
    passedValue.UpdateTime = System.currentTimeMillis() - 1000; //set it to 1 second ago

    when(limelightHigh.getRawTargetValid()).thenReturn(null);
    when(limelightLow.getRawTargetValid()).thenReturn(passedValue);
    
    var command = getCommand();

    //act
    var actual = command.getTargetAcquired();

    //assert
    assertFalse("Target should not be acquired with last value 1 second old", actual);
  }
}