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

/**
 * ShooterSubsystem
 */
public class ShooterSubsystem extends SubsystemBase {

  // shooter one motor
  // shooter two motor
  // private final WPI_TalonSRX shooterMaster = new WPI_TalonSRX(DEVICE_ID_SHOOTER_MASTER);
  // private final WPI_TalonSRX shooterSlave = new WPI_TalonSRX(DEVICE_ID_SHOOTER_SLAVE);

  private final CANSparkMax shooterMaster = new CANSparkMax(DEVICE_ID_SHOOTER_MASTER, MotorType.kBrushless);
  private final CANSparkMax shooterSlave = new CANSparkMax(DEVICE_ID_SHOOTER_SLAVE, MotorType.kBrushless);

  private final CANPIDController shooterPIDController = shooterMaster.getPIDController();
  private final CANEncoder shooterEncoder = shooterMaster.getEncoder();

  private double targetVelocity;

  private final SimpleMotorFeedforward motorFeedForward = new SimpleMotorFeedforward(.0822, .136, .0866);

  // hood motor

  public ShooterSubsystem() {
    // TalonSRXConfiguration shooterTalonConfig = new TalonSRXConfiguration();
    // shooterTalonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    // shooterTalonConfig.neutralDeadband = 0.001;
    // shooterTalonConfig.slot0.kF = 0.0;
    // shooterTalonConfig.slot0.kP = .2;
    // shooterTalonConfig.slot0.kI = 0.0;
    // shooterTalonConfig.slot0.kD = 14.1;
    // shooterTalonConfig.slot0.integralZone = 400;
    // shooterTalonConfig.slot0.closedLoopPeakOutput = 1.0;
    // shooterTalonConfig.closedloopRamp = 0.1;
    // shooterTalonConfig.openloopRamp = 0;

    // shooterMaster.configAllSettings(shooterTalonConfig);
    // shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    // shooterMaster.setSensorPhase(true);

    // shooterSlave.configOpenloopRamp(0);

    // shooterSlave.setInverted(InvertType.OpposeMaster);
    // shooterSlave.follow(shooterMaster);

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
    // targetVelocity = (41.772 * Units.metersToInches(distanceToTarget)) + 23000;
    targetVelocity = 4000;
    shooterPIDController.setReference(targetVelocity, ControlType.kVelocity,
        0, motorFeedForward.calculate(targetVelocity / 60));
    SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
  }

  public boolean isReadyToShoot() {
    return Math.abs(shooterEncoder.getVelocity() - targetVelocity) <= CLOSED_LOOP_ERROR_RANGE;
  }

  public void stopShooter() {
    //shooterPIDController.setReference(0d, ControlType.kVelocity);
    shooterMaster.set(0.0);
  }

  public void setProfile(Profile profile) {

  }

}