package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Unit tests for DriveTrainSubSystem
 */
public class DriveTrainSubsystemTest {

  @Test
  public void testEdgesToMeters() {
    assertEquals(5.777606101631194, DriveTrainSubsystem.edgesToMeters(12357), 0);
  }

  @Test
  public void testEdgesPerDecisecToMetersPerSec() {
    assertEquals(32.85993014668935, DriveTrainSubsystem.edgesPerDecisecToMetersPerSec(7028), 0);
  }
  
  @Test
  public void testMetersToEdges() {
    assertEquals(3089.25, DriveTrainSubsystem.metersToEdges(1.4444015254077984d), 0);
  }

  @Test
  public void testMetersPerSecToEdgesPerDecisec() {
    assertEquals(7028d, DriveTrainSubsystem.metersPerSecToEdgesPerDecisec(32.85993014668935), 0);
  }

}