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
    intakeMotor.configFactoryDefault();
  }

  public void intake() {
    if (isIndexerReady.get()) {
      intakeMotor.set(0.2);
    } else {
      stopIntake();
    }
  }

  public void reverse() {
    intakeMotor.set(-0.5);
  }

  public void stopIntake() {
    intakeMotor.set(0.0);
  }
  
}