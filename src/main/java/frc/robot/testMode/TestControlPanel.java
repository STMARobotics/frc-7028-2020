package frc.robot.testMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * Test the control panel manipulator.
 * <ol>
 * <li>Raise the arm to make sure it's zeroed</li>
 * <li>Lower the arm</li>
 * <li>Raise the arm again since this is starting configuration</li>
 * <li>Spin the spinner</li>
 * </ol>
 */
public class TestControlPanel extends TestCommand {

  private final ControlPanelSubsystem controlPanelSubsystem;
  private final NetworkTableEntry zeroedEntry =  getTestModeEntry("ArmTest_Zeroed");
  private final NetworkTableEntry loweredEntry = getTestModeEntry("ArmTest_Lowered");
  private final NetworkTableEntry raisedEntry = getTestModeEntry("ArmTest_Raised");
  private final NetworkTableEntry spunEntry = getTestModeEntry("ArmTest_Spun");

  private int testState;
  private double spinnerStartPosition;
  private Timer spinTimer = new Timer();

  public TestControlPanel(ControlPanelSubsystem controlPanelSubsystem) {
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  @Override
  public void initialize() {
    testState = 0;
    spinTimer.reset();
    zeroedEntry.setBoolean(false);
    loweredEntry.setBoolean(false);
    raisedEntry.setBoolean(false);
    spunEntry.setBoolean(false);
  }

  @Override
  public void execute() {
    switch (testState) {
      case 0: // Zero arm
        controlPanelSubsystem.raiseArm();
        if (controlPanelSubsystem.isArmUp()) {
          zeroedEntry.setBoolean(controlPanelSubsystem.getArmPosition() == 0);
          controlPanelSubsystem.stopArm();
          testState++;
        }
        break;
      case 1: // Move arm down
        controlPanelSubsystem.lowerArm();
        if (controlPanelSubsystem.isArmDown()) {
          // Make sure arm stops within 250 edges below the soft stop
          var position = controlPanelSubsystem.getArmPosition();
          loweredEntry.setBoolean(position <= ControlPanelConstants.ARM_DOWN_POSITION
              && position >= (ControlPanelConstants.ARM_DOWN_POSITION - 250));
          controlPanelSubsystem.stopArm();
          testState++;
        }
        break;
      case 2: // Move arm back up
        controlPanelSubsystem.raiseArm();
        if (controlPanelSubsystem.isArmUp()) {
          raisedEntry.setBoolean(controlPanelSubsystem.getArmPosition() == 0);
          controlPanelSubsystem.stopArm();
          testState++;
          // Setup for spin test
          spinTimer.start();
          spinnerStartPosition = controlPanelSubsystem.getSpinnerPosition();
        }
        break;
      case 3: // Spin spinner
        controlPanelSubsystem.spinSpeed(60);
        if (spinTimer.hasElapsed(3)) {
          spunEntry.setBoolean(controlPanelSubsystem.getSpinnerPosition() > spinnerStartPosition);
          controlPanelSubsystem.stopWheel();
          testState++;
        }
      default:
        // Shouldn't get here so just stop
        controlPanelSubsystem.stopArm();
        controlPanelSubsystem.stopWheel();
      }
  }

  @Override
  public boolean isFinished() {
    return testState >= 4;
  }

  @Override
  public void end(boolean interrupted) {
    controlPanelSubsystem.stopArm();
    controlPanelSubsystem.stopWheel();
  }

}