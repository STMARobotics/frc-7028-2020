package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrain.DEVICE_ID_LEFT_MASTER;
import static frc.robot.Constants.DriveTrain.DEVICE_ID_LEFT_SLAVE;
import static frc.robot.Constants.DriveTrain.DEVICE_ID_RIGHT_MASTER;
import static frc.robot.Constants.DriveTrain.DEVICE_ID_RIGHT_SLAVE;
import static frc.robot.Constants.DriveTrain.SENSOR_UNITS_PER_ROTATION;
import static frc.robot.Constants.DriveTrain.WHEEL_CIRCUMFERENCE_INCHES;
import static frc.robot.Constants.DriveTrain.WHEEL_CIRCUMFERENCE_METERS;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Auto;
import frc.robot.Constants.DriveTrain;

/**
 * DriveTrainSubsystem
 */
public class DriveTrainSubsystem extends SubsystemBase {

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DEVICE_ID_LEFT_MASTER);
  private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(DEVICE_ID_LEFT_SLAVE);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(DEVICE_ID_RIGHT_MASTER);
  private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(DEVICE_ID_RIGHT_SLAVE);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(
      new SpeedControllerGroup(leftMaster, leftSlave), new SpeedControllerGroup(rightMaster, rightSlave));

  private static final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry differentialDriveOdometry;
  private Pose2d savedPosition;

  public DriveTrainSubsystem() {
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getYaw()));
    savedPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(gyro.getYaw()));

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.neutralDeadband = 0.001;
    talonConfig.slot0.kF = 1023.0 / 6800.0;
    talonConfig.slot0.kP = 1.0;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.openloopRamp = .25;

    rightMaster.configAllSettings(talonConfig);
    leftMaster.configAllSettings(talonConfig);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    setNeutralMode(NeutralMode.Brake);

    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(true);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    differentialDrive.setRightSideInverted(false);
  }

  @Override
  public void periodic() {
    differentialDriveOdometry.update(Rotation2d.fromDegrees(gyro.getAngle()), stepsToMeters(getLeftEncoderPosition()),
        stepsToMeters(getRightEncoderPosition()));
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed    speed along the x axis [-1.0..1.0]
   * @param rotation rotation rate along the z axis [-1.0..1.0]
   */
  public void arcadeDrive(double speed, double rotation) {
    arcadeDrive(speed, rotation, false);
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed      speed along the x axis [-1.0..1.0]
   * @param rotation   rotation rate along the z axis [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void arcadeDrive(double speed, double rotation, boolean useSquares) {
    differentialDrive.arcadeDrive(speed, rotation, useSquares);
  }

  /**
   * Drives the robot by individually addressing the left and right side of the
   * drive train
   * 
   * @param leftSpeed  speed of the left motors [-1.0..1.0]
   * @param rightSpeed speed of the right motors [-1.0..1.0]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false);
  }

  /**
   * Drives the robot by individually addressing the left and right side of the
   * drive train
   * 
   * @param leftSpeed  speed of the left motors [-1.0..1.0]
   * @param rightSpeed speed of the right motors [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean useSquares) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed, useSquares);
  }

  /**
   * Sets the neutral mode for the drive train
   * 
   * @param neutralMode the desired neutral mode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    leftMaster.setNeutralMode(neutralMode);
    leftSlave.setNeutralMode(neutralMode);
    rightMaster.setNeutralMode(neutralMode);
    rightSlave.setNeutralMode(neutralMode);
  }

  public void setUseDifferentialDrive(boolean useDifferentialDrive) {
    differentialDrive.setSafetyEnabled(useDifferentialDrive);
    if (!useDifferentialDrive) {
      leftSlave.follow(leftMaster);
      rightSlave.follow(rightMaster);
    }
  }

  public WPI_TalonSRX getLeftTalonSRX() {
    return leftMaster;
  }

  public WPI_TalonSRX getRightTalonSRX() {
    return rightMaster;
  }

  /**
   * returns left encoder position
   * 
   * @return left encoder position
   */
  public int getLeftEncoderPosition() {
    return leftMaster.getSelectedSensorPosition(0);
  }

  /**
   * returns right encoder position
   * 
   * @return right encoder position
   */
  public int getRightEncoderPosition() {
    return rightMaster.getSelectedSensorPosition(0);
  }

  public void zeroDriveTrainEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public Pose2d getCurrentPose() {
    return differentialDriveOdometry.getPoseMeters();
  }

  public void saveCurrentPose() {
    savedPosition = getCurrentPose();
  }

  public Pose2d getSavedPose() {
    return savedPosition;
  }

  public float getYaw() {
    return gyro.getYaw();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    leftSlave.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    rightSlave.setVoltage(-rightVolts);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity()),
        stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity()));
  }

  public Command createRamseteCommandForTrajectory(Trajectory trajectory) {
    return new RamseteCommand(
        trajectory, 
        this::getCurrentPose,
        new RamseteController(Auto.RAMSETE_B, Auto.RAMSETE_ZETA),
        new SimpleMotorFeedforward(
            DriveTrain.STATIC_VOLTS,
            DriveTrain.VOLT_SECONDS_PER_METER,
            DriveTrain.VOLT_SECONDS_SQUARED_PER_METER),
        DriveTrain.DRIVE_KINEMATICS,
        this::getWheelSpeeds,
        new PIDController(DriveTrain.P_GAIN_DRIVE_VEL, 0, 0),
        new PIDController(DriveTrain.P_GAIN_DRIVE_VEL, 0, 0),
        this::tankDriveVolts,
        this);
  }

  /**
   * Converts inches to wheel revolutions
   * 
   * @param inches inches
   * @return wheel revolutions
   */
  public static double insToRevs(double inches) {
    return inches / WHEEL_CIRCUMFERENCE_INCHES;
  }

  /**
   * Converts inches to encoder steps
   * 
   * @param inches inches
   * @return encoder steps
   */
  public static double insToSteps(double inches) {
    return (insToRevs(inches) * SENSOR_UNITS_PER_ROTATION);
  }

  /**
   * Converts inches per second to encoder steps per decisecond
   * 
   * @param inchesPerSec inches per second
   * @return encoder steps per decisecond (100 ms)
   */
  public static double insPerSecToStepsPerDecisec(double inchesPerSec) {
    return insToSteps(inchesPerSec) * .1;
  }

  /**
   * Converts from encoder steps to meters.
   * 
   * @param steps encoder steps to convert
   * @return meters
   */
  public static double stepsToMeters(int steps) {
    return (WHEEL_CIRCUMFERENCE_METERS / SENSOR_UNITS_PER_ROTATION) * steps;
  }

  public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

}