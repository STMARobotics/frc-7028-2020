package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.DEVICE_ID_INTAKE;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * IntakeSubsystem
 */
public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(DEVICE_ID_INTAKE);
  private final Supplier<Boolean> isIndexerReady;

  public IntakeSubsystem(Supplier<Boolean> isIndexerReady) {
    this.isIndexerReady = isIndexerReady;
  }

  public void intake() {
    isIndexerReady.get();
    if (isIndexerReady.get()) {
      intakeMotor.set(1.0);
    } else {
      intakeMotor.set(0.0);
    }
  }

  public void reverse() {
    intakeMotor.set(-1.0);
  }
  
}