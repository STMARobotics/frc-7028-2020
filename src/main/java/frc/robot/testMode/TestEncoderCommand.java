package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TestEncoderCommand extends TestCommand {

  private final double power;
  private final DriveTrainSubsystem driveTrain;

  private int leftEncoder = 0, rightEncoder = 0, leftEncoderStart, rightEncoderStart;

  NetworkTableEntry leftSideEntry, rightSideEntry;
  
  public TestEncoderCommand (double power, DriveTrainSubsystem driveTrain) {
    this.power = power;
    this.driveTrain = driveTrain;

    addRequirements(driveTrain);

    var direction = "Reverse";
    if (power > 0) {
      direction = "Forward";
    }

    leftSideEntry = getTestModeEntry("EncoderTest_LeftSide_" + direction);
    rightSideEntry = getTestModeEntry("EncoderTest_RightSide_" + direction);
  }

  @Override
  public void initialize() {
    super.initialize();

    leftEncoderStart = driveTrain.getLeftEncoderPosition();
    rightEncoderStart = driveTrain.getRightEncoderPosition();
  }

  @Override
  public void execute() {
    super.execute();

    driveTrain.tankDriveRaw(power, power);

    assertEncoderPositions();
  }

  private void assertEncoderPositions() {

    var previousleft = leftEncoder;
    var previousRight = rightEncoder;

    leftEncoder = driveTrain.getLeftEncoderPosition();
    rightEncoder = driveTrain.getRightEncoderPosition();

    assertEncoderPosition(leftSideEntry, previousleft, leftEncoder);
    assertEncoderPosition(rightSideEntry, previousRight, rightEncoder);
  }

  private void assertEncoderPosition(NetworkTableEntry tableEntry, int previous, int current){
    
    //moving forward
    if (this.power > 0) {
      tableEntry.setBoolean(current <= previous);
    } else if (this.power < 0) {
      tableEntry.setBoolean(current >= previous);      
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    if (this.power > 0) {
      leftSideEntry.setBoolean(leftEncoder - rightEncoderStart > 1000);
      rightSideEntry.setBoolean(rightEncoder - rightEncoderStart > 1000);
    } else if (this.power < 0) {
      leftSideEntry.setBoolean(leftEncoderStart - leftEncoder > 1000);
      rightSideEntry.setBoolean(rightEncoderStart - rightEncoder < 1000);      
    }

    //stop drivetrain
    driveTrain.tankDriveRaw(0, 0);  
  }
}