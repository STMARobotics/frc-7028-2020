package frc.robot.testMode;

import java.util.ArrayList;
import java.util.List;

import frc.robot.commands.WaitForTargetCommand;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTest implements ITestable {
  private LimelightSubsystem limelight;
  private final WaitForTargetCommand waitCommand;
  private boolean isFinished;

  public LimelightTest(LimelightSubsystem limelight) {
    super();

    this.limelight = limelight;

    waitCommand = new WaitForTargetCommand(limelight);
    limelight.enable();
  }

  @Override
  public List<TestResult> testablePeriodic() {
    
    var result = new ArrayList<TestResult>();

    waitCommand.execute();

    if (waitCommand.isFinished()) {
      
      var message = limelight.limelightConfig.getNetworkTableName() +
        " target acquired. x:" + limelight.getTargetX() + ",y:" + limelight.getTargetY();

      result.add(new TestResult(true, message));
      isFinished = true;
    }

    return result;
  }

  @Override
  public boolean testableIsFinished() {
    return isFinished;
  }
}