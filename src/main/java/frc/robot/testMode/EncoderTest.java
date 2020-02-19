package frc.robot.testMode;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class EncoderTest implements ITestable {

  private int iterations = 0;
  private int encoderReading = 0;
  private int iterationsUnchanged = 0;
  private boolean isFinished = false;

  private final String testName;
  private final WPI_TalonSRX controller;
  private final int maxIterations;

  public EncoderTest(WPI_TalonSRX controller, int maxIterations, String testName) {
    super();

    controller.setSafetyEnabled(false);
    this.controller = controller;
    this.maxIterations = maxIterations;
    this.testName = testName;
  }

  @Override
  public List<TestResult> testablePeriodic() {
    var result = new ArrayList<TestResult>();

    var currentEncoderReading = controller.getSelectedSensorPosition();
    //go forward for half of iterations
    if (++iterations < maxIterations / 2)
    {
      controller.set(.25);

      //if current reading is less than previous encoder is out of phase
      if (currentEncoderReading < encoderReading)
      {
        result.add(new TestResult(testName + ": moving forward, encoder is out of phase"));
      }
    }
    else if (iterations - (maxIterations / 2) < 10) //deadzone for 10 iterations
    {
      controller.set(0);
    }
    else if (iterations < maxIterations)
    {
      controller.set(-.25);

      if (currentEncoderReading > encoderReading)
      {
        result.add(new TestResult(testName + ": moving backward, encoder is out of phase"));
      }
    }
    else //we hit our max iterations, report a success
    {
      result.add(new TestResult(true, testName + " test succeeded"));
    }

    if (currentEncoderReading == encoderReading) {
      iterationsUnchanged++;
    }

    if (iterationsUnchanged > 25)
    {
      result.add(new TestResult(testName + ": encoder is unchanged after 25 iterations, failing test"));
    }

    encoderReading = currentEncoderReading;
    
    isFinished = result.size() > 0;

    if (isFinished) {
      controller.set(0);
    }
    
    return result;
  }

  @Override
  public boolean testableIsFinished() {
    
    return isFinished;
  }
}