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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.JustShootCommand;
import frc.robot.commands.LowerArmCommand;
import frc.robot.commands.PIDPixyAssistCommand;
import frc.robot.commands.RaiseArmCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.WaitForTargetCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
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
  private final ControlPanelSubsystem controlPanelSubsystem;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public AutoGenerator(DriveTrainSubsystem driveTrainSubsystem, LimelightSubsystem highLimelightSubsystem,
      LimelightSubsystem lowLimelightSubsystem, IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, PixyVisionSubsystem pixyVision, ControlPanelSubsystem controlPanelSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.highLimelightSubsystem = highLimelightSubsystem;
    this.lowLimelightSubsystem = lowLimelightSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pixyVision = pixyVision;
    this.controlPanelSubsystem = controlPanelSubsystem;
  }

  public void configureAutonomous() {
    configureCenterTrenchAuto();
    configureMoveAuto();
    configureRightTrenchAuto();
    configureShieldGeneratorAuto();
    configureStealAuto();
  }

  public void addDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);
  }

  private void configureCenterTrenchAuto() {
    try {
      var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(-95), Rotation2d.fromDegrees(0));

      var autoCommandGroup = makeShootPathTrenchShoot(125, startPose);
        
      autoChooser.setDefaultOption("Center Trench", autoCommandGroup);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto: Center Trench", true);
    }
  }

  private void configureRightTrenchAuto() {
    try {
      var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(-42), Rotation2d.fromDegrees(20));

      var autoCommandGroup = makeShootPathTrenchShoot(140, startPose);
        
      autoChooser.addOption("Right Trench", autoCommandGroup);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto: Right Trench", true);
    }
  }

  /**
   * Makes a sequence that shoots 3 balls a given distance, drives from startPose to the trench, then picks up 3 balls
   * in the trench and shoots.
   * 
   * @param distanceToTarget starting distance to the target
   * @param startPose starting pose of the bot
   * @param trenchPose ending pose of the bot
   * @return command group
   */
  private Command makeShootPathTrenchShoot( int distanceToTarget, Pose2d startPose) {
    var trenchPose = new Pose2d(inchesToMeters(178), inchesToMeters(-42), Rotation2d.fromDegrees(0));
    var trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        Collections.emptyList(),
        trenchPose,
        new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO * .5)
            .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
            .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
            .setEndVelocity(TrajectoryConstants.MAX_SPEED_AUTO * 0.2));

    var pickupAndSpinUp = makePixyAutoCommand()
        .andThen(makeWaitForBallCount(3).withTimeout(3))
        .andThen(new PrintCommand("Done with ball pick up"))
        .andThen(new TurnToAngleCommand(10, driveTrainSubsystem))
        .andThen(new PrintCommand("Done turning to angle"))
        .andThen(new WaitForTargetCommand(highLimelightSubsystem, lowLimelightSubsystem).withTimeout(5))
        .andThen(new PrintCommand("Found target"))
        .deadlineWith(new SpinUpShooterCommand(180, shooterSubsystem));

    var trenchPickup = makePixyAutoCommand()
        .andThen(makePixyAutoCommand())
        .andThen(pickupAndSpinUp)
        .deadlineWith(
            new RunIntakeCommand(intakeSubsystem, indexerSubsystem::isFull),
            new IndexCommand(indexerSubsystem))
        .andThen(driveTrainSubsystem::stop, driveTrainSubsystem);

    return new RaiseArmCommand(controlPanelSubsystem)
        .andThen(new LowerArmCommand(controlPanelSubsystem))
        .andThen(() -> indexerSubsystem.resetBallCount(3))
        .andThen(()-> driveTrainSubsystem.setCurrentPose(startPose), driveTrainSubsystem)
        .deadlineWith(new SpinUpShooterCommand(distanceToTarget, shooterSubsystem))
        .andThen(new PrintCommand("About to shoot 3"))
        .andThen(new JustShootCommand(3, distanceToTarget, shooterSubsystem, indexerSubsystem))
        .andThen(new PrintCommand("Shot 3"))
        .andThen(makeLimelightProfileCommand(Profile.FAR))
        .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectory))
        .andThen(new PrintCommand("Drove to trench"))
        .andThen(trenchPickup)
        .andThen(new PrintCommand("Done with trench pickup, ready to shoot"))
        .andThen(makeShootCommand(3))
        .andThen(new PrintCommand("Done shooting, all done"));
  }

  private void configureShieldGeneratorAuto() {
    try {
      var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(-175), Rotation2d.fromDegrees(0));
      var endPose = new Pose2d(inchesToMeters(235), inchesToMeters(-187), Rotation2d.fromDegrees(22.5));      

      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPose,
          Collections.emptyList(),
          endPose,
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
              .setEndVelocity(TrajectoryConstants.MAX_SPEED_AUTO * 0.6));

      var startPoseTwo = new Pose2d(inchesToMeters(240), inchesToMeters(-175), Rotation2d.fromDegrees(0));
      var endPoseTwo = new Pose2d(inchesToMeters(204), inchesToMeters(-163), Rotation2d.fromDegrees(22.5));    

      var trajectoryTwo = TrajectoryGenerator.generateTrajectory(
        startPoseTwo,
        Collections.emptyList(),
        endPoseTwo,
        new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
            .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
            .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
            .setReversed(true));

      var startPoseThree = new Pose2d(inchesToMeters(240), inchesToMeters(-160), Rotation2d.fromDegrees(22.5));
      var endPoseThree = new Pose2d(inchesToMeters(2228), inchesToMeters(-163), Rotation2d.fromDegrees(-15));    
      
      var trajectoryThree = TrajectoryGenerator.generateTrajectory(
        startPoseThree,
        Collections.emptyList(),
        endPoseThree,
        new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
            .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
            .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
            .setReversed(true));

      var autoCommandGroup = new RaiseArmCommand(controlPanelSubsystem)
          .andThen(new LowerArmCommand(controlPanelSubsystem))
          .alongWith(new InstantCommand(() -> indexerSubsystem.resetBallCount(3))
              .andThen(()-> driveTrainSubsystem.setCurrentPose(trajectory.getInitialPose()), driveTrainSubsystem)
              .andThen(makeLimelightProfileCommand(Profile.FAR))
              .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectory))
              .andThen(makePixyWithIntakeCommand())
              .andThen(makeWaitForBallCount(4).withTimeout(3))
              .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectoryTwo))
              .andThen(makePixyWithIntakeCommand())
              .andThen(makeWaitForBallCount(5).withTimeout(3)))
          .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectoryThree)
              .andThen(new WaitForTargetCommand(highLimelightSubsystem, lowLimelightSubsystem).withTimeout(5))
              .deadlineWith(new SpinUpShooterCommand(180, shooterSubsystem)))
          .andThen(makeShootCommand(5));
        
      autoChooser.addOption("Shield Generator", autoCommandGroup);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto: Shield Generator", true);
    }
  }

  private void configureStealAuto() {
    try {
      var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(-296), Rotation2d.fromDegrees(0));
      var endPose = new Pose2d(inchesToMeters(240), inchesToMeters(-296), Rotation2d.fromDegrees(0));      

      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPose,
          Collections.emptyList(),
          endPose,
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
              .setEndVelocity(TrajectoryConstants.MAX_SPEED_AUTO * 0.6));

      var startPoseTwo = new Pose2d(inchesToMeters(250), inchesToMeters(-296), Rotation2d.fromDegrees(0));
      var endPoseTwo = new Pose2d(inchesToMeters(240), inchesToMeters(-296), Rotation2d.fromDegrees(-62));    

      var trajectoryTwo = TrajectoryGenerator.generateTrajectory(
          startPoseTwo,
          Collections.emptyList(),
          endPoseTwo,
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
              .setReversed(true));

      var startPoseThree = new Pose2d(inchesToMeters(250), inchesToMeters(-305), Rotation2d.fromDegrees(-62));
      var endPoseThree = new Pose2d(inchesToMeters(180), inchesToMeters(-162), Rotation2d.fromDegrees(-35));    
      
      var trajectoryThree = TrajectoryGenerator.generateTrajectory(
          startPoseThree,
          Collections.emptyList(),
          endPoseThree,
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
              .setReversed(true));

      var autoCommandGroup = new RaiseArmCommand(controlPanelSubsystem)
          .andThen(new LowerArmCommand(controlPanelSubsystem))
          .alongWith(new InstantCommand(() -> indexerSubsystem.resetBallCount(3))
              .andThen(()-> driveTrainSubsystem.setCurrentPose(trajectory.getInitialPose()), driveTrainSubsystem)
              .andThen(makeLimelightProfileCommand(Profile.FAR))
              .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectory))
              .andThen(makePixyWithIntakeCommand())
              .andThen(makeWaitForBallCount(4).withTimeout(3))
              .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectoryTwo))
              .andThen(makePixyWithIntakeCommand())
              .andThen(makeWaitForBallCount(5).withTimeout(3)))
          .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectoryThree)
              .andThen(new WaitForTargetCommand(highLimelightSubsystem, lowLimelightSubsystem).withTimeout(5))
              .deadlineWith(new SpinUpShooterCommand(180, shooterSubsystem)))
          .andThen(makeShootCommand(5));
        
      autoChooser.addOption("Steal", autoCommandGroup);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto: steal", true);
    }
  }

  private void configureMoveAuto() {
    try {
      var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(0), Rotation2d.fromDegrees(0));
      var endPose = new Pose2d(inchesToMeters(180), inchesToMeters(0), Rotation2d.fromDegrees(0));

      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPose,
          Collections.emptyList(),
          endPose,
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO * .5, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT)
              .setEndVelocity(0));

      var autoCommandGroup = new RaiseArmCommand(controlPanelSubsystem)
          .andThen(new LowerArmCommand(controlPanelSubsystem))
          .alongWith(new InstantCommand(() -> indexerSubsystem.resetBallCount(3))
              .andThen(()-> driveTrainSubsystem.setCurrentPose(startPose), driveTrainSubsystem))
          .andThen(new PrintCommand("About to shoot 3"))
          .andThen(new JustShootCommand(3, 130, shooterSubsystem, indexerSubsystem))
          .andThen(new PrintCommand("Shot 3"))
          .andThen(driveTrainSubsystem.createCommandForTrajectory(trajectory))
          .andThen(new PrintCommand("Drove out of the way"))
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
    return new PIDPixyAssistCommand(driveTrainSubsystem, pixyVision)
    .andThen(new PrintCommand("Found ball with Pixy"))
        .andThen(new RunCommand(() -> driveTrainSubsystem.arcadeDrive(.3, 0, false), driveTrainSubsystem).withTimeout(0.25))
        .andThen(new PrintCommand("Pixy thinks it captured a ball"))
        .andThen(driveTrainSubsystem::stop);
  }

  private Command makePixyWithIntakeCommand() {
    return makePixyAutoCommand().deadlineWith(
        new RunCommand(intakeSubsystem::intake, intakeSubsystem),
        new IndexCommand(indexerSubsystem)).andThen(intakeSubsystem::stopIntake, intakeSubsystem);
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
    return new InstantCommand(() -> {
        highLimelightSubsystem.setProfile(profile);
        lowLimelightSubsystem.setProfile(profile);
    }, highLimelightSubsystem, lowLimelightSubsystem);
  }

  /**
   * Makes a command to wait for at least a given ball count of balls inthe indexer or for the indexer to be full.
   * @param ballCount minimun number of balls to wait for
   * @return command
   */
  private Command makeWaitForBallCount(int ballCount) {
    return new WaitUntilCommand(() -> indexerSubsystem.getBallCount() >= ballCount || indexerSubsystem.isFull());
  }

  /**
   * Gets the selected autonomous command
   * @return
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}