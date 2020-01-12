package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Unit tests for DriveTrainSubSystem
 */
public class DriveTrainSubsystemTest {

  @Test
  public void testInsPerSecToStepsPerDecisec() {
    assertEquals(1023.1187024816481d, DriveTrainSubsystem.insPerSecToStepsPerDecisec(47.083333d), 0d);
  }

  @Test
  public void testInchesToRevolutions() {
    assertEquals(1.0610329539459689051258917558168, DriveTrainSubsystem.insToRevs(20), 0);
  }

  @Test
  public void testInsToSteps() {
    assertEquals(26825.629320116195601979665869943d, DriveTrainSubsystem.insToSteps(123.45d), 0);
  }

  @Test
  public void testStepsToMeters() {
    assertEquals(1.4444015254077984d, DriveTrainSubsystem.stepsToMeters(12357), 0);
  }

  @Test
  public void testStepsPerDecisecToMetersPerSec() {
    assertEquals(8.214982536672338, DriveTrainSubsystem.stepsPerDecisecToMetersPerSec(7028), 0);
  }

}