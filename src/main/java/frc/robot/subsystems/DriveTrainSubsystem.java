package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrain.DEVICE_ID_LEFT_MASTER;
import static frc.robot.Constants.DriveTrain.DEVICE_ID_LEFT_SLAVE;
import static frc.robot.Constants.DriveTrain.DEVICE_ID_RIGHT_MASTER;
import static frc.robot.Constants.DriveTrain.DEVICE_ID_RIGHT_SLAVE;
import static frc.robot.Constants.DriveTrain.SENSOR_UNITS_PER_ROTATION;
import static frc.robot.Constants.DriveTrain.WHEEL_CIRCUMFERENCE_INCHES;
import static frc.robot.Constants.DriveTrain.WHEEL_CIRCUMFERENCE_METERS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry differentialDriveOdometry;
  private Pose2d savedPose;

  public DriveTrainSubsystem() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));    

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
    rightMaster.overrideLimitSwitchesEnable(false);
    leftMaster.overrideLimitSwitchesEnable(false);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    differentialDrive.setRightSideInverted(false);
  }

  public void resetOdometry() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading()));
    differentialDriveOdometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    differentialDriveOdometry.update(
        Rotation2d.fromDegrees(getHeading()),
        stepsToMeters(getLeftEncoderPosition()),
        stepsToMeters(getRightEncoderPosition()));
    SmartDashboard.putString("Pose", differentialDriveOdometry.getPoseMeters().toString());
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
    savedPose = getCurrentPose();
  }

  public Pose2d getSavedPose() {
    return savedPose;
  }

  /**
   * Returns the heading of the robot in form required for odometry.
   *
   * @return the robot's heading in degrees, from 180 to 180 with positive value
   *         for left turn.
   */
  private double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d;
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  /**
   * Controls the left and right side of the drive using  Talon SRX closed-loop velocity.
   * @param leftVelocity left velocity
   * @param rightVelocity right velocity
   */
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    leftMaster.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(leftVelocity));
    rightMaster.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(rightVelocity));
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity()),
        stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity()));
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory) {
    return new InstantCommand(() -> setUseDifferentialDrive(false), this)
        .andThen(new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(Auto.RAMSETE_B, Auto.RAMSETE_ZETA),
            DriveTrain.DRIVE_KINEMATICS,
            this::tankDriveVelocity,
            this))
        .andThen(new InstantCommand(() -> {
            setUseDifferentialDrive(true);
            arcadeDrive(0, 0);
        }, this));
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

  /**
   * Converts from encoder units per 100 milliseconds to meters per second.
   * @param stepsPerDecisec steps per decisecond
   * @return meters per second
   */
  public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

  /**
   * Converts from meters to encoder units.
   * @param meters meters
   * @return encoder units
   */
  public static double metersToSteps(double meters) {
    return (meters / WHEEL_CIRCUMFERENCE_METERS) * SENSOR_UNITS_PER_ROTATION;
  }

  /**
   * Convers from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

}