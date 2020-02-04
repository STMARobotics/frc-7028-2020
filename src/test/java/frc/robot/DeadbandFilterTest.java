package frc.robot;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * DeadbandFilterTest
 */
public class DeadbandFilterTest {

  @Test
  public void testInBand() {
    var filter = new DeadbandFilter(-0.1, 0.1);

    assertEquals(0, filter.calculate(0.09), 0.0);
    assertEquals(0, filter.calculate(-0.09), 0.0);
    assertEquals(0, filter.calculate(0), 0.0);
  }

  @Test
  public void testOutOfBand() {
    var filter = new DeadbandFilter(-0.1, 0.1);

    assertEquals(0.1, filter.calculate(0.1), 0.0);
    assertEquals(-0.1, filter.calculate(-0.1), 0.0);
    assertEquals(11, filter.calculate(11), 0.0);
  }

}