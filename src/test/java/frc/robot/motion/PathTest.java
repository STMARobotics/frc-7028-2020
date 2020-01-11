package frc.robot.motion;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * PathTest
 */
public class PathTest {

  private String origUserDir;

  @Before
  public void setUp() {
    origUserDir = System.getProperty("user.dir");
    System.setProperty("user.dir", "./src/test");
  }

  @After
  public void tearDown() {
    System.setProperty("user.dir", origUserDir);
  }

  @Test
  public void testLoadPath() {
    var path = Path.loadFromPathWeaver("TestPath");

    assertNotNull(path);
    assertEquals("Left trajectory wrong", 197, path.getLeftTrajectory().segments.length);
    assertEquals("Right trajectory wrong", 197, path.getRightTrajectory().segments.length);
  }
}