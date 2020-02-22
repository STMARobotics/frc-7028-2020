/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.wpilibj.util.Units.inchesToMeters;
import static frc.robot.Constants.ControllerConstants.PORT_ID_DRIVER_CONTROLLER;
import static frc.robot.Constants.ControllerConstants.PORT_ID_OPERATOR_CONSOLE;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.InstantWhenDisabledCommand;
import frc.robot.commands.PixyAssistCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.commands.TeleOperateCommand;
import frc.robot.commands.WaitForTargetCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.testMode.TestEncoderCommand;
import frc.robot.testMode.TestIndexerCommand;
import frc.robot.testMode.TestLimelightCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final LimelightSubsystem highLimelightSubsystem = new LimelightSubsystem(LimelightConfig.Builder.create()
      .withNetworkTableName(LimeLightConstants.HIGH_NAME).withMountDepth(LimeLightConstants.HIGH_DISTANCE_FROM_FRONT)
      .withMountingHeight(LimeLightConstants.HIGH_MOUNT_HEIGHT).withMountingAngle(LimeLightConstants.HIGH_MOUNT_ANGLE)
      .withMountDistanceFromCenter(LimeLightConstants.HIGH_DISTANCE_FROM_CENTER).build());
  private final LimelightSubsystem lowLimelightSubsystem = new LimelightSubsystem(LimelightConfig.Builder.create()
      .withNetworkTableName(LimeLightConstants.LOW_NAME).withMountDepth(LimeLightConstants.LOW_DISTANCE_FROM_FRONT)
      .withMountingHeight(LimeLightConstants.LOW_MOUNT_HEIGHT).withMountingAngle(LimeLightConstants.LOW_MOUNT_ANGLE)
      .withMountDistanceFromCenter(LimeLightConstants.LOW_DISTANCE_FROM_CENTER).build());
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(indexerSubsystem::isReadyForBall);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final PixyVisionSubsystem pixyVision = new PixyVisionSubsystem();

  private final XboxController driverController = new XboxController(PORT_ID_DRIVER_CONTROLLER);
  private final XboxController operatorConsole = new XboxController(PORT_ID_OPERATOR_CONSOLE);

  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(driverController, driveTrainSubsystem);
  private final TeleOperateCommand teleOperateCommand = new TeleOperateCommand(operatorConsole, indexerSubsystem);
  private final IndexCommand indexCommand = new IndexCommand(indexerSubsystem);
  private final ShootCommand shootCommand = new ShootCommand(Integer.MAX_VALUE, shooterSubsystem, indexerSubsystem, 
    highLimelightSubsystem, lowLimelightSubsystem, driveTrainSubsystem);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();
    configureSubsystemDashboard();
    // configureCommandDashboard();
    configureDriverDashboard();
    configureAutonomous();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whenPressed(teleDriveCommand::toggleSlowMode);

    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whenPressed(teleDriveCommand::toggleReverseMode);

    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whenHeld(shootCommand.perpetually());
    
    new JoystickButton(driverController, XboxController.Button.kBumperRight.value)
        .whenHeld(new RunCommand(() -> {
          intakeSubsystem.reverse();
          indexerSubsystem.reverse();
        }, intakeSubsystem))
        .whenReleased(() -> {
          intakeSubsystem.stopIntake();
          indexerSubsystem.stopIndexer();
        }, intakeSubsystem);

    new JoystickButton(driverController, XboxController.Button.kBumperLeft.value)
        .whenHeld(new RunCommand(intakeSubsystem::intake, intakeSubsystem))
        .whenReleased(intakeSubsystem::stopIntake, intakeSubsystem);

    var pixyHeldCommand = new PixyAssistCommand(driveTrainSubsystem, pixyVision)
        .andThen(new ParallelCommandGroup(
            new RunCommand(intakeSubsystem::intake, intakeSubsystem).withTimeout(.5),
            new RunCommand(() -> driveTrainSubsystem.arcadeDrive(.2, 0, false), driveTrainSubsystem).withTimeout(0.25))
        .andThen(new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem)
        .andThen(new InstantCommand(driveTrainSubsystem::stop, driveTrainSubsystem))));
    
    var pixyReleaseCommand = new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem)
        .andThen(new InstantCommand(driveTrainSubsystem::stop, driveTrainSubsystem));

    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileHeld(new ConditionalCommand(new StartEndCommand(() -> driverController.setRumble(RumbleType.kLeftRumble, 1)
        , () -> driverController.setRumble(RumbleType.kLeftRumble, 0)), pixyHeldCommand, indexerSubsystem::isFull))
        .whenReleased(pixyReleaseCommand);

    // Operator
    new JoystickButton(operatorConsole, XboxController.Button.kA.value)
        .whenHeld(new RunCommand(() -> indexerSubsystem.runManually(1.0), indexerSubsystem))
        .whenReleased(indexerSubsystem::stopIndexer, indexerSubsystem);

    new JoystickButton(operatorConsole, XboxController.Button.kB.value)
        .whenHeld(new RunCommand(() -> indexerSubsystem.runManually(-1.0), indexerSubsystem))
        .whenReleased(() -> indexerSubsystem.runManually(0.0), indexerSubsystem);

    new JoystickButton(operatorConsole, XboxController.Button.kBumperLeft.value)
        .whenPressed(makeLimelightProfileCommand(Profile.NEAR));
    
    new JoystickButton(operatorConsole, XboxController.Button.kBumperRight.value)
        .whenPressed(makeLimelightProfileCommand(Profile.FAR));
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
    indexerSubsystem.setDefaultCommand(indexCommand);
  }

  private void configureAutonomous() {
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
      DriverStation.reportError("Failed to load auto", true);
    }

    try {
      var trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          Collections.emptyList(),
          new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
          new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO / 2, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
              .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
              .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT));

      var autoCommandGroup =
          driveTrainSubsystem.createCommandForTrajectory(trajectory)
          .andThen(driveTrainSubsystem::stop, driveTrainSubsystem);
        
      autoChooser.setDefaultOption("Straight", autoCommandGroup);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load auto", true);
    }    
  }

  /**
   * Makes a command that will pick up one ball with the Pixy
   * @return command
   */
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

  private void configureSubsystemDashboard() {
    var drivetrainLayout = Dashboard.subsystemsTab.getLayout("Drivetrain", BuiltInLayouts.kList)
        .withSize(2, 5).withPosition(0, 0);
    driveTrainSubsystem.addDashboardWidgets(drivetrainLayout);
    drivetrainLayout.add(driveTrainSubsystem);

    var indexerLayout = Dashboard.subsystemsTab.getLayout("Indexer", BuiltInLayouts.kList)
        .withSize(2, 4).withPosition(2, 0);
    indexerSubsystem.addDashboardWidgets(indexerLayout);
    indexerLayout.add(indexerSubsystem);
    
    var shooterLayout = Dashboard.subsystemsTab.getLayout("Shooter", BuiltInLayouts.kList)
        .withSize(2,2).withPosition(4, 0);
    shooterSubsystem.addDashboardWidgets(shooterLayout);
    shooterLayout.add(shooterSubsystem);

    var highLimelightLayout = Dashboard.limelightsTab.getLayout("High Limelight", BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(0, 0);
    highLimelightSubsystem.addDashboardWidgets(highLimelightLayout);
    highLimelightLayout.add(highLimelightSubsystem);

    var lowLimelightLayout = Dashboard.limelightsTab.getLayout("Low Limelight", BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(2, 0);
    lowLimelightSubsystem.addDashboardWidgets(lowLimelightLayout);
    lowLimelightLayout.add(lowLimelightSubsystem);
  }

  private void configureCommandDashboard() {
    Dashboard.commandsTab.add(indexCommand).withSize(2, 1).withPosition(0, 0);
    Dashboard.commandsTab.add(shootCommand).withSize(2, 1).withPosition(0, 1);
    Dashboard.commandsTab.add(teleDriveCommand).withSize(2, 1).withPosition(2, 0);
  }

  private void configureDriverDashboard() {
    Dashboard.driverTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);

    var indexerLayout = Dashboard.driverTab.getLayout("Indexer", BuiltInLayouts.kGrid)
        .withSize(2, 1).withPosition(0, 1)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 1));
    indexerLayout.addBoolean("Full", indexerSubsystem::isFull);
    indexerLayout.addBoolean("Running", indexerSubsystem::isRunning);

    var lowLimelight = CameraServer.getInstance().getServer(LimeLightConstants.LOW_NAME);
    if (lowLimelight != null) {
      Dashboard.driverTab.add(lowLimelight.getSource()).withSize(2, 2).withPosition(1, 0);
    }

    var highLimelight = CameraServer.getInstance().getServer(LimeLightConstants.HIGH_NAME);
    if (highLimelight != null) {
      Dashboard.driverTab.add(highLimelight.getSource()).withSize(2, 2).withPosition(3, 0);
    }
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void resetOdometry() {
    new InstantCommand(driveTrainSubsystem::resetOdometry, driveTrainSubsystem).schedule();
  }

  Command[] getTestModeCommands() {
    var commands = new ArrayList<Command>();

    //add encoder commands, chain forward/reverse with a 1 second wait in between to allow the drivetrain to stop
    commands.add(new TestEncoderCommand(.25, driveTrainSubsystem).withTimeout(5)
      .andThen(new WaitCommand(1))
      //add a 5 second reverse test
      .andThen(new TestEncoderCommand(-.25, driveTrainSubsystem).withTimeout(5)));

    //add commands for each limelight system
    commands.add(new TestLimelightCommand(highLimelightSubsystem).withTimeout(10));
    commands.add(new TestLimelightCommand(lowLimelightSubsystem).withTimeout(10));

    commands.add(new TestIndexerCommand(indexerSubsystem).withTimeout(20));

    Command[] arr = new Command[commands.size()];
    return commands.toArray(arr);
  }
}
