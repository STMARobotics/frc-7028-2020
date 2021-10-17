package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.DEVICE_ID_INTAKE;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * IntakeSubsystem
 */
public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(DEVICE_ID_INTAKE);
  private final Supplier<Boolean> isIndexerReady;

  public IntakeSubsystem(Supplier<Boolean> isIndexerReady) {
    this.isIndexerReady = isIndexerReady;
    intakeMotor.configFactoryDefault();
    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public void intake() {
    if (isIndexerReady.get()) {
      intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    } else {
      intakeMotor.set(IntakeConstants.INTAKE_SPEED * .5);
    }
  }

  public void reverse() {
    intakeMotor.set(-0.5);
  }

  public void stopIntake() {
    intakeMotor.set(0.0);
  }

  public double getEncoderPosition() {
    return intakeMotor.getSelectedSensorPosition();
  }
  
}