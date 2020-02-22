package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.when;

import org.junit.BeforeClass;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.junit.MockitoJUnitRunner;

@RunWith(MockitoJUnitRunner.class)
public class IndexerSubsystemTest {
  private static IndexerSubsystem indexer;

  @BeforeClass
  public static void setUp() {
    //doing this in before class so the digital sensor API doesn't fail on port already being used
    indexer = spy(new IndexerSubsystem());
  }

  @Test
  public void testIntakeOuttakeSingleBall() {
    //arrange
    when(indexer.getBeltValue()).thenReturn(.5); //we're intaking

    //act
    indexer.spaceSensorTripped();

    //assert
    assertEquals(1, indexer.getBallCount());

    //reverse the belt
    when(indexer.getBeltValue()).thenReturn(-.5);

    indexer.spaceSensorCleared();

    assertEquals(0, indexer.getBallCount());
  }

  @Test
  public void testIntakeOuttake5Balls() {
    //arrange
    when(indexer.getBeltValue()).thenReturn(.5); //we're intaking

    //act
    for(var idx = 1; idx <= 5; idx++){
      indexer.spaceSensorTripped();

      assertEquals(idx, indexer.getBallCount());

      //this really shouldn't be needed, but represents what it would be like to have a ball move through..
      if (idx < 5) {
        indexer.spaceSensorCleared(); //this won't happen for the 5th ball
      }
    }
    
    //reverse the belt
    when(indexer.getBeltValue()).thenReturn(-.5);

    for(var idx = 4; idx >= 0; idx--) {
      
      indexer.spaceSensorCleared();// ejecting the 5th one first, which likely never cleared the sensor

      assertEquals(idx, indexer.getBallCount());
    }
  }

  @Test
  public void testIntakeShoot5Balls() {
    //arrange
    when(indexer.getBeltValue()).thenReturn(.5); //we're intaking

    //act
    for(var idx = 1; idx <= 5; idx++){
      indexer.spaceSensorTripped();

      assertEquals(idx, indexer.getBallCount());

      //this really shouldn't be needed, but represents what it would be like to have a ball move through..
      if (idx < 5) {
        indexer.spaceSensorCleared(); //this won't happen for the 5th ball
      }
    }
    
    //clear the full sensor 5 times
    for(var idx = 4; idx >= 0; idx--) {
      
      indexer.fullSensorCleared();// shooting a ball out

      assertEquals(idx, indexer.getBallCount());
    }
  }
}