package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climb subsystem
 */
public class ClimbSubsystem extends SubsystemBase {

  private final CANSparkMax climb = new CANSparkMax(DEVICE_ID_CLIMB, MotorType.kBrushless);

  public ClimbSubsystem() {
    climb.restoreFactoryDefaults();
    climb.setInverted(true);
    climb.setSoftLimit(SoftLimitDirection.kReverse, -40);
    climb.setSoftLimit(SoftLimitDirection.kForward, 497);
    climb.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climb.enableSoftLimit(SoftLimitDirection.kForward, true);
    climb.burnFlash();
  }

  public void addDashboardWidget(ShuffleboardLayout dashboard) {
    dashboard.addNumber("Position", () -> climb.getEncoder().getPosition());
  }

  public void raiseClimb() {
    climb.set(1);
  }

  public void lowerClimb() {
    climb.set(-0.9);
  }

  public void stopClimb() {
    climb.set(0);
  }

  public double getClimbPosition() {
    return climb.getEncoder().getPosition();
  }

}