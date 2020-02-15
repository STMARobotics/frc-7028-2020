package frc.robot.testMode;

import java.util.List;

public interface ITestable
{
  List<TestResult> testablePeriodic();
  boolean testableIsFinished();
}