package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climb subsystem
 */
public class ClimbSubsystem extends SubsystemBase {

  private final CANSparkMax climb = new CANSparkMax(DEVICE_ID_CLIMB, MotorType.kBrushless);

  public void raiseClimb() {
    climb.set(-1);
  }

  public void lowerClimb() {
    climb.set(0.9);
  }

  public void stopClimb() {
    climb.set(0);
  }

}