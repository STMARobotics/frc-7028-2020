/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.ControllerConstants.PORT_ID_DRIVER_CONTROLLER;
import static frc.robot.Constants.ControllerConstants.PORT_ID_OPERATOR_CONSOLE;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.PixyAssistCommand;
import frc.robot.commands.RotateWheelCommand;
import frc.robot.commands.SetColorCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.commands.TeleOperateCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;

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
  private final ControlPanelSubsystem controlPanelSubsystem = new ControlPanelSubsystem();
  private final LimelightSubsystem highLimelightSubsystem = new LimelightSubsystem(LimelightConfig.Builder.create()
      .withNetworkTableName("limelight-high").withMountDepth(LimeLightConstants.HIGH_DISTANCE_FROM_FRONT)
      .withMountingHeight(LimeLightConstants.HIGH_MOUNT_HEIGHT).withMountingAngle(LimeLightConstants.HIGH_MOUNT_ANGLE)
      .withMountDistanceFromCenter(LimeLightConstants.HIGH_DISTANCE_FROM_CENTER).build());
  private final LimelightSubsystem lowLimelightSubsystem = new LimelightSubsystem(LimelightConfig.Builder.create()
      .withNetworkTableName("limelight-low").withMountDepth(LimeLightConstants.LOW_DISTANCE_FROM_FRONT)
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
  private final ShootCommand shootCommand = new ShootCommand(
      shooterSubsystem, 
      indexerSubsystem, 
      highLimelightSubsystem, 
      lowLimelightSubsystem, 
      driveTrainSubsystem);
  
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private RotateWheelCommand rotateWheelCommand = new RotateWheelCommand(controlPanelSubsystem);
  private SetColorCommand setColorCommand = new SetColorCommand(controlPanelSubsystem);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();
    configureSubsystemDashboard();
    configureCommandDashboard();

    try {
      var straightTrajectory = loadTrajectory("Straight");
      Transform2d transform = new Pose2d(0, 0, Rotation2d.fromDegrees(0)).minus(straightTrajectory.getInitialPose());
      Trajectory newTrajectory = straightTrajectory.transformBy(transform);
      var straightPathCommand = driveTrainSubsystem.createCommandForTrajectory(newTrajectory);
      autoChooser.setDefaultOption("Straight", straightPathCommand);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: Straight", false);
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
        .whenHeld(new RunCommand(intakeSubsystem::intake, intakeSubsystem))
        .whenReleased(intakeSubsystem::stopIntake, intakeSubsystem);

    new JoystickButton(driverController, XboxController.Button.kBumperLeft.value)
        .whenHeld(new RunCommand(intakeSubsystem::reverse, intakeSubsystem))
        .whenReleased(intakeSubsystem::stopIntake, intakeSubsystem);

    new JoystickButton(driverController, XboxController.Button.kBumperLeft.value)
        .whenHeld(new RunCommand(indexerSubsystem::reverse, indexerSubsystem))
        .whenReleased(indexerSubsystem::stopIndexer, indexerSubsystem);

    var pixyHeldCommand = new PixyAssistCommand(driveTrainSubsystem, pixyVision)
        .andThen(new RunCommand(intakeSubsystem::intake, intakeSubsystem).withTimeout(1))
        .alongWith(new RunCommand(() -> driveTrainSubsystem.arcadeDrive(.25, 0, false), driveTrainSubsystem).withTimeout(1));
    
    var pixyReleaseCommand = new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem)
        .andThen(new InstantCommand(driveTrainSubsystem::stop, driveTrainSubsystem));

    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whenHeld(pixyHeldCommand)
        .whenReleased(pixyReleaseCommand);

    // Operator
    new JoystickButton(operatorConsole, XboxController.Button.kA.value)
        .whenHeld(new RunCommand(() -> indexerSubsystem.runManually(1.0), indexerSubsystem))
        .whenReleased(indexerSubsystem::stopIndexer, indexerSubsystem);

    new JoystickButton(operatorConsole, XboxController.Button.kB.value)
        .whenHeld(new RunCommand(() -> indexerSubsystem.runManually(-1.0), indexerSubsystem))
        .whenReleased(() -> indexerSubsystem.runManually(0.0), indexerSubsystem);

    new JoystickButton(operatorConsole, XboxController.Button.kX.value)
        .whenHeld(rotateWheelCommand);

    new JoystickButton(operatorConsole, XboxController.Button.kStart.value)
        .whenHeld(setColorCommand);

    new JoystickButton(operatorConsole, XboxController.Button.kBumperLeft.value)
        .whenPressed(() -> {
            highLimelightSubsystem.setProfile(Profile.NEAR);
            lowLimelightSubsystem.setProfile(Profile.NEAR);
        });
    
    new JoystickButton(operatorConsole, XboxController.Button.kBumperRight.value)
        .whenPressed(() -> {
            highLimelightSubsystem.setProfile(Profile.FAR);
            lowLimelightSubsystem.setProfile(Profile.FAR);
        });
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
    indexerSubsystem.setDefaultCommand(indexCommand);
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

    var controlPanelLayout = Dashboard.subsystemsTab.getLayout("Control Panel", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 5)).withSize(2, 5).withPosition(6, 0);
    controlPanelSubsystem.addDashboardWidgets(controlPanelLayout);
    controlPanelLayout.add(controlPanelSubsystem);

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

    var rotateWheelLayout = 
        Dashboard.commandsTab.getLayout("Rotate Wheel", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
    rotateWheelCommand.addDashboardWidgets(rotateWheelLayout);
    rotateWheelLayout.add(rotateWheelCommand);
    
    var setColorLayout =
        Dashboard.commandsTab.getLayout("Set Color", BuiltInLayouts.kList).withSize(2, 2).withPosition(6, 0);
    setColorCommand.addDashboardWidgets(setColorLayout);
    setColorLayout.add(setColorCommand);
    
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
}
