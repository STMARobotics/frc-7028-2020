package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterHoodConstants;

/**
 * ShooterHoodSubsystem
 */
public class ShooterHoodSubsystem extends SubsystemBase{
  private final Servo hood = new Servo(ShooterHoodConstants.AcutatorPort);
  private final double maxValue = ShooterHoodConstants.Maximum;
  private final double minValue = ShooterHoodConstants.Minimum;

  public ShooterHoodSubsystem() {
    super();

    //took these from the documentation here: 
    //https://andymark-weblinc.netdna-ssl.com/media/W1siZiIsIjIwMTkvMDMvMjIvMTAvMjcvNTgvMDMxOTQ4ODUtYmM5Yi00M2UyLWE1NDAtZGNiMWVhNzEzMDEzL1VzaW5nIEwxNiBMaW5lYXIgU2Vydm8gMDMtMjAxOS5wZGYiXV0/Using%20L16%20Linear%20Servo%2003-2019.pdf?sha=ee4c9607408cc835
    hood.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    new Trigger(() -> RobotState.isDisabled()).whenActive(() -> { this.setPercentage(0);});
  }
  
  /**
   * Set speed between configurd min/max values
   * @param speed Raw speed value to set between min and max allowed
   */
  public void setSpeed(double speed)
  {
    //wrapping this in a min/max check
    hood.setSpeed(Math.max(minValue, Math.min(speed, maxValue)));
  }

  /**
   * Set the actuator to a percentage of the allowed range
   * @param percent 1 = max extension, 0 = full retraction
   */
  public void setPercentage(double percent)
  {
    setSpeed(percentageToSpeed(percent, minValue, maxValue));
  }

  /**
   * Calculate the speed given a percentage and min/max
   * @param percent Desired percentage of allowed range
   * @param minimumValue Minimum allowed point of retraction (default is -1)
   * @param maximumValue Maximum allowed point of extension (default is 1)
   * @return Speed to provide to setSpeed method
   */
  public static double percentageToSpeed(double percent, double minimumValue, double maximumValue)
  {
    //maximum travel is 2 if we don't falsely limit the min/max
    var totalAllowedTravel = maximumValue - minimumValue;

    var desiredTravelFromMin = totalAllowedTravel * Math.max(0, Math.min(percent, 1));

    return desiredTravelFromMin + minimumValue;
  }
}