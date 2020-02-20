package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Unit tests for DriveTrainSubSystem
 */
public class DriveTrainSubsystemTest {

  @Test
  public void testEdgesToMeters() {
    assertEquals(0.7222007627038992, DriveTrainSubsystem.edgesToMeters(12357), 0);
  }

  @Test
  public void testEdgesPerDecisecToMetersPerSec() {
    assertEquals(4.107491268336169, DriveTrainSubsystem.edgesPerDecisecToMetersPerSec(7028), 0);
  }

  @Test
  public void testMetersToEdges() {
    assertEquals(24714.0, DriveTrainSubsystem.metersToEdges(1.4444015254077984d), 0);
  }

  @Test
  public void testMetersPerSecToEdgesPerDecisec() {
    assertEquals(7028d, DriveTrainSubsystem.metersPerSecToEdgesPerDecisec(4.107491268336169), 0);
  }

} 