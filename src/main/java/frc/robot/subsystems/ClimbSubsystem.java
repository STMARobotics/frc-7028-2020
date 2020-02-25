package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ClimbSubsystem
 */
public class ClimbSubsystem extends SubsystemBase {

  private final WPI_TalonSRX climb = new WPI_TalonSRX(DEVICE_ID_CLIMB);

  public ClimbSubsystem() {

  }

  public void driveClimb(double speed) {
    climb.set(speed);
  }

  public void stopClimb() {
    climb.set(0);
  }

}