/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.testMode.EncoderTest;
import frc.robot.testMode.ITestable;
import frc.robot.testMode.TestResult;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    robotContainer.resetOdometry();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.resetOdometry();
  }

  @Override
  public void teleopPeriodic() {
  }

  private List<ITestable> testables;
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    
    var testModeTable = NetworkTableInstance.getDefault().getTable("Test Mode");

    for (String key: testModeTable.getKeys()) {
      testModeTable.delete(key);
    }

    testables = new ArrayList<ITestable>();

    testables.add(new EncoderTest(robotContainer.driveTrainSubsystem.leftMaster, 200, "Left Encoder"));
    testables.add(new EncoderTest(robotContainer.driveTrainSubsystem.rightMaster, 200, "Right Encoder"));
  }

  @Override
  public void testPeriodic() {
    
    for (ITestable testable : testables) {
      var results = testable.testablePeriodic();

      for (TestResult testResult : results) {
        NetworkTableInstance.getDefault().getTable("Test Mode").getEntry(testResult.message).setValue(testResult.success);
      }

      if (testable.testableIsFinished())
      {
        testables.remove(testable);
      }
    }
  }
}