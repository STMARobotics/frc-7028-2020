package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.CLOSED_LOOP_ERROR_RANGE;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_MASTER;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_SLAVE;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final SimpleMotorFeedforward motorFeedForward = 
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

  public ShooterSubsystem() {
    shooterPIDController.setP(.0002);
    shooterPIDController.setI(0);
    shooterPIDController.setD(0);
    shooterPIDController.setIZone(400);
    shooterPIDController.setFF(0);
    shooterPIDController.setOutputRange(-1.0, 1.0);

    shooterSlave.follow(shooterMaster, true);

    shooterMaster.setClosedLoopRampRate(.2);
    shooterSlave.setClosedLoopRampRate(.2);
  }

  public void prepareToShoot(double distanceToTarget) {
    shooterPIDController.setReference(ShooterConstants.SHOOTER_RPM, ControlType.kVelocity,
        0, motorFeedForward.calculate(ShooterConstants.SHOOTER_RPM / 60));
    SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
  }

  public boolean isReadyToShoot() {
    return Math.abs(shooterEncoder.getVelocity() - ShooterConstants.SHOOTER_RPM) <= CLOSED_LOOP_ERROR_RANGE;
  }

  public void stopShooter() {
    shooterMaster.set(0.0);
  }

  public void setProfile(Profile profile) {

  }

}