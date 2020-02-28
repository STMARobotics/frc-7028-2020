package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.CLOSED_LOOP_ERROR_RANGE;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_MASTER;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_SLAVE;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * ShooterSubsystem
 */
public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax shooterMaster = new CANSparkMax(DEVICE_ID_SHOOTER_MASTER, MotorType.kBrushless);
  private final CANSparkMax shooterSlave = new CANSparkMax(DEVICE_ID_SHOOTER_SLAVE, MotorType.kBrushless);

  private final CANPIDController shooterPIDController = shooterMaster.getPIDController();
  private final CANEncoder shooterEncoder = shooterMaster.getEncoder();

  private double nearGain = 0;
  private double farGain = 0;
  private double targetSpeed;

  private final SimpleMotorFeedforward motorFeedForward = 
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

  public ShooterSubsystem() {
    shooterMaster.restoreFactoryDefaults();
    shooterSlave.restoreFactoryDefaults();

    shooterPIDController.setP(ShooterConstants.kP);
    shooterPIDController.setIZone(400);
    shooterPIDController.setOutputRange(-1.0, 1.0);

    shooterMaster.setIdleMode(IdleMode.kCoast);

    shooterSlave.setIdleMode(IdleMode.kCoast);
    shooterSlave.follow(shooterMaster, true);

    shooterMaster.setClosedLoopRampRate(ShooterConstants.RAMP_RATE);
    shooterMaster.enableVoltageCompensation(12);
    shooterSlave.enableVoltageCompensation(12);
    shooterMaster.burnFlash();
    shooterSlave.burnFlash();
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    dashboard.addNumber("Velocity", shooterEncoder::getVelocity);
  }

  public void addDriverDashboardWidget(ShuffleboardLayout dashboard) {
    var nearGainEntry = dashboard.addPersistent("Near Gain", 0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Max", 100, "Min", -100)).getEntry();
    nearGainEntry.addListener(this::updateNearGain, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    
    var farGainEntry = dashboard.addPersistent("Far Gain", 0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Max", 100, "Min", -100)).getEntry();
    farGainEntry.addListener(this::updateFarGain, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  private void updateNearGain(EntryNotification notification) {
    nearGain = notification.value.getDouble();
  }

  private void updateFarGain(EntryNotification notification) {
    farGain = notification.value.getDouble();
  }

  /**
   * Prepare to shoot the given distance in INCHES
   * @param distanceToTarget distance in INCHES
   */
  public void prepareToShoot(double distanceToTarget) {
    if (distanceToTarget > 150) {
      targetSpeed = 3.05 * distanceToTarget + (2160.761 + farGain);
    } else if (distanceToTarget <= 150) {
      targetSpeed = .25 * Math.pow(distanceToTarget, 2) - 75.833 * distanceToTarget + (8420 + nearGain);
    }
    shooterPIDController.setReference(
        targetSpeed,
        ControlType.kVelocity,
        0,
        motorFeedForward.calculate(targetSpeed / 60, (targetSpeed - shooterEncoder.getVelocity()) / 60));
  }

  public boolean isReadyToShoot() {
    return Math.abs(shooterEncoder.getVelocity() - targetSpeed) <= CLOSED_LOOP_ERROR_RANGE;
  }

  public void stopShooter() {
    shooterMaster.set(0.0);
  }

  public void setProfile(Profile profile) {

  }

  public double getVelocity() {
    return shooterEncoder.getVelocity();
  }

  public double getTargetSpeed() {
    return targetSpeed;
  }

}