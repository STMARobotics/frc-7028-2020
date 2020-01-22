package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_MASTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_SLAVE;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_MASTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_SLAVE;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;
import static frc.robot.Constants.DriveTrainConstants.SENSOR_UNITS_PER_ROTATION;
import static frc.robot.Constants.DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS;
import static frc.robot.Constants.TeleConstants.MAX_ANGULAR_VEL;
import static frc.robot.Constants.TeleConstants.MAX_SPEED_TELE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;

/**
 * DriveTrainSubsystem
 */
public class DriveTrainSubsystem extends SubsystemBase {

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DEVICE_ID_LEFT_MASTER);
  private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(DEVICE_ID_LEFT_SLAVE);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(DEVICE_ID_RIGHT_MASTER);
  private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(DEVICE_ID_RIGHT_SLAVE);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry differentialDriveOdometry;
  private Pose2d savedPose;

  public DriveTrainSubsystem() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.neutralDeadband = DriveTrainConstants.DEADBAND;
    talonConfig.slot0.kP = DriveTrainConstants.kP;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = DriveTrainConstants.kD;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.closedloopRamp = DriveTrainConstants.CLOSED_LOOP_RAMP;
    talonConfig.openloopRamp = DriveTrainConstants.OPEN_LOOP_RAMP;

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
    var xSpeed = speed * MAX_SPEED_TELE;
    var zRotation = Rotation2d.fromDegrees(-rotation * MAX_ANGULAR_VEL).getRadians();;
    if (useSquares) {
      xSpeed *= Math.abs(xSpeed);
      zRotation *= Math.abs(zRotation);
    }
    var wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, zRotation));
    tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
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
    var xLeftSpeed = leftSpeed * MAX_SPEED_TELE;
    var xRightSpeed = rightSpeed * MAX_SPEED_TELE;
    if (useSquares) {
      xLeftSpeed *= Math.abs(xLeftSpeed);
      xRightSpeed *= Math.abs(xRightSpeed);
    }
    tankDriveVelocity(xLeftSpeed, xRightSpeed);
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

  private void zeroDriveTrainEncoders() {
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
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d;
  }

  /**
   * Controls the left and right side of the drive using Talon SRX closed-loop
   * velocity.
   * 
   * @param leftVelocity  left velocity
   * @param rightVelocity right velocity
   */
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity())) / 20;
    var rightAccel = (rightVelocity - stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity())) / 20;
    
    var leftFeedForwardVolts = FEED_FORWARD.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = FEED_FORWARD.calculate(rightVelocity, rightAccel);

    leftMaster.set(
        ControlMode.Velocity, 
        metersPerSecToStepsPerDecisec(leftVelocity), 
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
    rightMaster.set(
        ControlMode.Velocity,
        metersPerSecToStepsPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory) {
    return new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(TrajectoryConstants.RAMSETE_B, TrajectoryConstants.RAMSETE_ZETA),
            DriveTrainConstants.DRIVE_KINEMATICS,
            this::tankDriveVelocity,
            this)
        .andThen(new InstantCommand(() -> arcadeDrive(0, 0), this));
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