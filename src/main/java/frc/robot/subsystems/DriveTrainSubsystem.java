package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.kToggleSwitch;
import static frc.robot.Constants.ArcadeConstants.MAX_ANGULAR_VEL_ARCADE;
import static frc.robot.Constants.ArcadeConstants.MAX_SPEED_ARCADE;
import static frc.robot.Constants.ArcadeConstants.ROTATE_RATE_LIMIT_ARCADE;
import static frc.robot.Constants.ArcadeConstants.SPEED_RATE_LIMIT_ARCADE;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_MASTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_SLAVE;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_MASTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_SLAVE;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;
import static frc.robot.Constants.DriveTrainConstants.SENSOR_UNITS_PER_ROTATION;
import static frc.robot.Constants.DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Dashboard;

/**
 * DriveTrainSubsystem
 */
public class DriveTrainSubsystem extends SubsystemBase {

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DEVICE_ID_LEFT_MASTER);
  private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(DEVICE_ID_LEFT_SLAVE);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(DEVICE_ID_RIGHT_MASTER);
  private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(DEVICE_ID_RIGHT_SLAVE);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry differentialDriveOdometry;
  private Pose2d savedPose;

  private SlewRateLimiter speedRateLimiter = new SlewRateLimiter(SPEED_RATE_LIMIT_ARCADE);
  private SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATE_RATE_LIMIT_ARCADE);

  private final ShuffleboardLayout dashboard = Dashboard.subsystemsTab.getLayout("Drivetrain", BuiltInLayouts.kList)
      .withSize(2, 4).withPosition(0, 0);
  private NetworkTableEntry useEncodersEntry =
      dashboard.addPersistent("Use encoders", true).withWidget(kToggleSwitch).getEntry();
  
  public DriveTrainSubsystem() {
    dashboard.add(this);

    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    dashboard.add(rightMaster);
    dashboard.add(leftMaster);
    dashboard.addString("Pose", () -> differentialDriveOdometry.getPoseMeters().toString());

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.slot0.kP = DriveTrainConstants.kP;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
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

    differentialDrive.setRightSideInverted(false);
  }

  /**
   * Resets the current pose to 0, 0, 0° and resets the saved pose
   */
  public void resetOdometry() {
    zeroDriveTrainEncoders();
    savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    differentialDriveOdometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    differentialDriveOdometry.update(
        Rotation2d.fromDegrees(getHeading()),
        stepsToMeters(getLeftEncoderPosition()),
        stepsToMeters(getRightEncoderPosition()));
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed      speed along the x axis [-1.0..1.0]
   * @param rotation   rotation rate along the z axis [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void arcadeDrive(double speed, double rotation, boolean useSquares) {
    if(useEncodersEntry.getBoolean(true)) {
      var xSpeed = speedRateLimiter.calculate(safeClamp(speed));
      var zRotation = -rotationRateLimiter.calculate(safeClamp(rotation));
      if (useSquares) {
        xSpeed *= Math.abs(xSpeed);
        zRotation *= Math.abs(zRotation);
      }
      xSpeed *= MAX_SPEED_ARCADE;
      zRotation *= MAX_ANGULAR_VEL_ARCADE;
      var wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, zRotation));
      tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    } else {
      differentialDrive.arcadeDrive(speed, rotation, useSquares);
    }
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
    if(useEncodersEntry.getBoolean(true)) {
      var xLeftSpeed = safeClamp(leftSpeed) * MAX_SPEED_ARCADE;
      var xRightSpeed = safeClamp(rightSpeed) * MAX_SPEED_ARCADE;
      if (useSquares) {
        xLeftSpeed *= Math.abs(xLeftSpeed);
        xRightSpeed *= Math.abs(xRightSpeed);
      }
      tankDriveVelocity(xLeftSpeed, xRightSpeed);
    } else {
      differentialDrive.tankDrive(leftSpeed, rightSpeed, useSquares);
    }
  }

  /**
   * Controls the left and right side of the drive using Talon SRX closed-loop
   * velocity.
   * 
   * @param leftVelocity  left velocity in meters per second
   * @param rightVelocity right velocity in meters per second
   */
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity())) / .20;
    var rightAccel = (rightVelocity - stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity())) / .20;
    
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
    differentialDrive.feed();
  }

  /**
   * Sets the drivetrain to zero velocity and rotation.
   */
  public void stop() {
    tankDriveVelocity(0, 0);
  }

  /**
   * Returns value clamped between [-1, 1]. Not-a-number (NaN) returns 0;
   * @param input value to clamp
   * @return clamped value
   */
  private double safeClamp(double input) {
    if (Double.isNaN(input)) {
      return 0;
    }
    return MathUtil.clamp(input, -1, 1);
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
   * @return the robot's heading in degrees, from -180 to 180 with positive value
   *         for left turn.
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d;
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
        .andThen(this::stop, this);
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