package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

/**
 * PathTest
 */
public class RobotContainerTest {

  @Test
  public void testLoadTrajectory() throws Exception {
    var origUserDir = System.getProperty("user.dir");
    try {
      System.setProperty("user.dir", "./src/test");
      
      var path = RobotContainer.loadTrajectory("TestPath");

      assertNotNull(path);
      assertEquals(4.378882294106705, path.getTotalTimeSeconds(), 0);
    } finally {
      System.setProperty("user.dir", origUserDir);
    }
  }
}