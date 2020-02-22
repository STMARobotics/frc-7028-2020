package frc.robot;

import static edu.wpi.first.wpilibj.util.Units.inchesToMeters;

import java.util.Collections;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.InstantWhenDisabledCommand;
import frc.robot.commands.PixyAssistCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.WaitForTargetCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * AutoGenerator
 */
public class AutoGenerator {
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final LimelightSubsystem highLimelightSubsystem;
  private final LimelightSubsystem lowLimelightSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PixyVisionSubsystem pixyVision;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public AutoGenerator(DriveTrainSubsystem driveTrainSubsystem, LimelightSubsystem highLimelightSubsystem,
      LimelightSubsystem lowLimelightSubsystem, IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, PixyVisionSubsystem pixyVision) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.highLimelightSubsystem = highLimelightSubsystem;
    this.lowLimelightSubsystem = lowLimelightSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pixyVision = pixyVision;
  }

  public void configureAutonomous() {
    configureShootingAuto();
    configureMoveAuto();
    configureRightAuto();
  }

  public void addDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);
  }

  private void configureShootingAuto() {
    try {
      var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(-95), Rotation2d.fromDegrees(0));
      var endPose = new Pose2d(inchesToMeters(178), inchesToMeters(-36), Rotation2d.fromDegrees(0));

      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPose,
          Collections.emptyList(),
          endPose,
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
              .setEndVelocity(TrajectoryConstants.MAX_SPEED_AUTO * 0.6));

      var pickupAndSpinUp = makePixyAutoCommand()
          .andThen(new WaitUntilCommand(() -> indexerSubsystem.getBallCount() >= 3).withTimeout(3))
          .deadlineWith(new RunCommand(() -> shooterSubsystem.prepareToShoot(180), shooterSubsystem));

      var trenchPickup = makePixyAutoCommand()
          .andThen(makePixyAutoCommand())
          .andThen(pickupAndSpinUp)
          .deadlineWith(
              new RunCommand(intakeSubsystem::intake, intakeSubsystem),
              new IndexCommand(indexerSubsystem))
          .andThen(intakeSubsystem::stopIntake);

      var autoCommandGroup =
          new InstantCommand(() -> indexerSubsystem.resetBallCount(3))
              .andThen(()-> driveTrainSubsystem.setCurrentPose(trajectory.getInitialPose()), driveTrainSubsystem)
              .andThen(makeLimelightProfileCommand(Profile.NEAR))
              .andThen(new WaitForTargetCommand(highLimelightSubsystem, lowLimelightSubsystem).withTimeout(5))
              .andThen(makeShootCommand(3))
              .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectory))
              .andThen(makeLimelightProfileCommand(Profile.FAR))
              .andThen(trenchPickup)
              .andThen(intakeSubsystem::stopIntake, intakeSubsystem)
              .andThen(driveTrainSubsystem::stop, driveTrainSubsystem)
              .andThen(new WaitForTargetCommand(highLimelightSubsystem, lowLimelightSubsystem).withTimeout(5))
              .andThen(makeShootCommand(3));
        
      autoChooser.setDefaultOption("Shooting", autoCommandGroup);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto: shooting", true);
    }
  }

  private void configureRightAuto() {
    try {
      var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(-36), Rotation2d.fromDegrees(26));
      var endPose = new Pose2d(inchesToMeters(178), inchesToMeters(-36), Rotation2d.fromDegrees(0));      

      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPose,
          Collections.emptyList(),
          endPose,
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
              .setEndVelocity(TrajectoryConstants.MAX_SPEED_AUTO * 0.6));

      var startPoseTwo = new Pose2d(inchesToMeters(379), inchesToMeters(-26), Rotation2d.fromDegrees(0));
      var endPoseTwo = new Pose2d(inchesToMeters(315), inchesToMeters(-36), Rotation2d.fromDegrees(20));    

      var trajectoryTwo = TrajectoryGenerator.generateTrajectory(
        startPoseTwo,
        Collections.emptyList(),
        endPoseTwo,
        new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
            .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
            .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
            .setEndVelocity(TrajectoryConstants.MAX_SPEED_AUTO * 0.6));

      var trenchPickup = makePixyAutoCommand()
          .andThen(makePixyAutoCommand())
          .andThen(makePixyAutoCommand())
          .andThen(makePixyAutoCommand())
          .andThen(new WaitUntilCommand(() -> indexerSubsystem.getBallCount() >= 4).withTimeout(3))
          .deadlineWith(
              new RunCommand(intakeSubsystem::intake, intakeSubsystem),
              new IndexCommand(indexerSubsystem))
          .andThen(intakeSubsystem::stopIntake);

      var initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), Rotation2d.fromDegrees(0));

      var autoCommandGroup =
          new InstantCommand(() -> indexerSubsystem.resetBallCount(3))
              .andThen(()-> driveTrainSubsystem.setCurrentPose(initialPose), driveTrainSubsystem)
              .andThen(makeLimelightProfileCommand(Profile.NEAR))
              .andThen(new TurnToAngleCommand(26, driveTrainSubsystem))
              .andThen(new WaitForTargetCommand(highLimelightSubsystem, lowLimelightSubsystem).withTimeout(5))
              .andThen(makeShootCommand(3))
              .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectory))
              .andThen(makeLimelightProfileCommand(Profile.FAR))
              .andThen(trenchPickup)
              .andThen(intakeSubsystem::stopIntake, intakeSubsystem)
              .andThen(driveTrainSubsystem::stop, driveTrainSubsystem)
              .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectoryTwo)
                  .andThen(new WaitForTargetCommand(highLimelightSubsystem, lowLimelightSubsystem).withTimeout(5)))
                  .deadlineWith(new RunCommand(() -> shooterSubsystem.prepareToShoot(180), shooterSubsystem))
              .andThen(makeShootCommand(4));
        
      autoChooser.setDefaultOption("Right Auto", autoCommandGroup);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto: right", true);
    }
  }

  private void configureMoveAuto() {
    try {
      var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(0), Rotation2d.fromDegrees(0));
      var endPose = new Pose2d(inchesToMeters(96), inchesToMeters(0), Rotation2d.fromDegrees(0));

      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPose,
          Collections.emptyList(),
          endPose,
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
              .setEndVelocity(0));

      var autoCommandGroup =
          new InstantCommand(() -> indexerSubsystem.resetBallCount(3))
              .andThen(()-> driveTrainSubsystem.setCurrentPose(trajectory.getInitialPose()), driveTrainSubsystem)
              .andThen(makeLimelightProfileCommand(Profile.NEAR))
              .andThen(new WaitForTargetCommand(highLimelightSubsystem, lowLimelightSubsystem).withTimeout(5))
              .andThen(makeShootCommand(3))
              .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectory))
              .andThen(driveTrainSubsystem::stop, driveTrainSubsystem);
        
      autoChooser.addOption("Move", autoCommandGroup);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto: move", true);
    }
  }

  private Command makePixyAutoCommand() {
    // Pixy until ball within target and then drive for 250ms
    // At the same time, run the intake and indexer
    // Finally, stop the intake
    return new PixyAssistCommand(driveTrainSubsystem, pixyVision)
        .andThen(new RunCommand(() -> driveTrainSubsystem.arcadeDrive(.3, 0, false), driveTrainSubsystem).withTimeout(0.5))
        .andThen(driveTrainSubsystem::stop);
  }

  /**
   * Makes a command to shoot the specified number of balls
   * @param ballsToShoot number of balls to shoot
   * @return command
   */
  private Command makeShootCommand(int ballsToShoot) {
    return new ShootCommand(ballsToShoot, shooterSubsystem, indexerSubsystem, highLimelightSubsystem, 
        lowLimelightSubsystem, driveTrainSubsystem);
  }

  /**
   * Makes a command to select the limelight profile
   * @param profile profile to select
   * @return command
   */
  private Command makeLimelightProfileCommand(Profile profile) {
    return new InstantWhenDisabledCommand(() -> {
        highLimelightSubsystem.setProfile(profile);
        lowLimelightSubsystem.setProfile(profile);
    }, highLimelightSubsystem, lowLimelightSubsystem);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}