package frc.robot.commands;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.mockito.Mockito.when;

import org.junit.Before;
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

  private VisionCommandBase command;

  @Before
  public void init() {
    
    command = new VisionCommandBase(limelightHigh, limelightLow);
  }

  @Test
  public void testNullTargetValidValue() {
    //arrange
    when(limelightHigh.getRawTargetValid()).thenReturn(null);
    when(limelightLow.getRawTargetValid()).thenReturn(null);
    
    //act
    var actual = command.getTargetAcquired();

    //assert
    assertNull("Target shouldn't be acquired with null values", actual);
  }

  @Test
  public void testOneTargetValidValue() {
    //arrange
    when(limelightHigh.getRawTargetValid()).thenReturn(null);
    when(limelightLow.getRawTargetValid()).thenReturn(new DoubleEntryValue(Constants.LimeLightConstants.TARGET_ACQUIRED));
    
    //act
    var actual = command.getTargetAcquired();

    //assert
    assertNotNull("Target should be acquired with one value", actual);
  }

  @Test
  public void testTargetValidValueFalseDelayNotPassed() {
    //arrange
    when(limelightHigh.getRawTargetValid()).thenReturn(null);
    when(limelightLow.getRawTargetValid()).thenReturn(new DoubleEntryValue(0.0));
    
    //act
    var actual = command.getTargetAcquired();

    //assert
    assertNotNull("Target should be acquired with 500 ms delay in effect", actual);
  }

  @Test
  public void testTargetValidValueFalseDelayPassed() {
    //arrange
    var passedValue = new DoubleEntryValue(0.0);
    passedValue.updateTime = System.currentTimeMillis() - 1000; //set it to 1 second ago

    when(limelightHigh.getRawTargetValid()).thenReturn(null);
    when(limelightLow.getRawTargetValid()).thenReturn(passedValue);
    
    //act
    var actual = command.getTargetAcquired();

    //assert
    assertNull("Target should not be acquired with last value 1 second old", actual);
  }
}