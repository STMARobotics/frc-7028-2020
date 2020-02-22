package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.IndexerSubsystem;

public class TestIndexerCommand extends TestCommand
{
  private IndexerSubsystem indexer;
  private boolean reverse = false;

  private NetworkTableEntry ballCountIntake;
  private NetworkTableEntry ballCountReverse;
  private NetworkTableEntry fullSensorEntry;

  public TestIndexerCommand(IndexerSubsystem indexer) {
    super();

    this.indexer = indexer;
    addRequirements(indexer);

    fullSensorEntry = getTestModeEntry("IndexerFullSensor");
    ballCountIntake = getTestModeEntry("BallCountIntakeTest");
    ballCountReverse = getTestModeEntry("BallCountReverseTest");
  }

  @Override
  public void execute() {
    super.execute();

    //as long as we're moving forward set this value to true once ball count gets above 0
    if (!reverse ) { 
      //if we're moving forward set this based on ball count
      ballCountIntake.setBoolean(indexer.getBallCount() > 0);
    
      if (indexer.getBallCount() == 0) {
        indexer.intake();
        return;
      }

      if (!indexer.isFull()) {
        indexer.runManually(.5);
        return;
      } 
      else {
        reverse = true;
        fullSensorEntry.setBoolean(true);
      }
    }

    //if we're reversing and ballcount greater than zero run the belt backward
    if (reverse && indexer.getBallCount() > 0) {
      indexer.runManually(-.5);
    }
  }

  @Override
  public boolean isFinished() {
    var isFinished = reverse && indexer.getBallCount() == 0;

    //if isFinished is true then we decremented the counter and we're done here
    ballCountReverse.setBoolean(isFinished);
   
    return isFinished;
  }
}