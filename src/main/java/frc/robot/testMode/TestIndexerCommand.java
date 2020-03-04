package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.IndexerSubsystem;

public class TestIndexerCommand extends TestCommand {

  private IndexerSubsystem indexer;
  private boolean reverse = false;
  private int encoderStartPosition;

  private NetworkTableEntry ballCountIntake;
  private NetworkTableEntry ballCountReverse;
  private NetworkTableEntry fullSensorEntry;
  private NetworkTableEntry encoderEntry;

  public TestIndexerCommand(IndexerSubsystem indexer) {
    super();

    this.indexer = indexer;
    addRequirements(indexer);

    fullSensorEntry = getTestModeEntry("IndexerFullSensor");
    fullSensorEntry.setBoolean(false);

    ballCountIntake = getTestModeEntry("BallCountIntakeTest");
    ballCountIntake.setBoolean(false);
    
    ballCountReverse = getTestModeEntry("BallCountReverseTest");
    ballCountReverse.setBoolean(false);

    encoderEntry = getTestModeEntry("IndexerEncoder");
    encoderEntry.setBoolean(false);
  }

  @Override
  public void initialize() {
    super.initialize();
    
    encoderStartPosition = indexer.getPosition();
  }

  @Override
  public void execute() {
    super.execute();

    //as long as we're moving forward set this value to true once ball count gets above 0
    if (!reverse ) { 
      //if we're moving forward set this based on ball count
      ballCountIntake.setBoolean(indexer.getBallCount() > 0);
    
      if (indexer.getBallCount() == 0) {
        indexer.intake(); //this will run the belt only if both
        return;
      }

      if (!indexer.isFull()) {
        indexer.runManually(.5);
        return;
      } 
      
      //indexer is full let's reverse
      reverse = true;
      fullSensorEntry.setBoolean(true);
      encoderEntry.setBoolean(indexer.getPosition() - encoderStartPosition > 1000);
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