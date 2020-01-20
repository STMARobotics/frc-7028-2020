package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Unit tests for DriveTrainSubSystem
 */
public class DriveTrainSubsystemTest {

  @Test
  public void testStepsToMeters() {
    assertEquals(1.4444015254077984d, DriveTrainSubsystem.stepsToMeters(12357), 0);
  }

  @Test
  public void testStepsPerDecisecToMetersPerSec() {
    assertEquals(8.214982536672338, DriveTrainSubsystem.stepsPerDecisecToMetersPerSec(7028), 0);
  }
  
  @Test
  public void testMetersToSteps() {
    assertEquals(12357, DriveTrainSubsystem.metersToSteps(1.4444015254077984d), 0);
  }

  @Test
  public void testMetersPerSecToStepsPerDecisec() {
    assertEquals(7028, DriveTrainSubsystem.metersPerSecToStepsPerDecisec(8.214982536672338), 0);
  }

}