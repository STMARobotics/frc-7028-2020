package frc.robot.testMode;

public class TestResult
{
  public boolean success;
  public String message;

  public TestResult(String message) {
    this(false, message);
  }

  public TestResult(boolean success, String message) {
    super();
    
    this.success = success;
    this.message = message;
  }
}