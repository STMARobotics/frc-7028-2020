/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.ControllerConstants.PORT_ID_DRIVER_CONTROLLER;
import static frc.robot.Constants.ControllerConstants.PORT_ID_OPERATOR_CONSOLE;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.TrajectoryConstants.MAX_ACCELERATION_AUTO;
import static frc.robot.Constants.TrajectoryConstants.MAX_SPEED_AUTO;
import static frc.robot.Constants.TrajectoryConstants.VOLTAGE_CONSTRAINT;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Collections;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.AimShooterCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.RotateWheelCommand;
import frc.robot.commands.SetColorCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.commands.TeleOperateCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.LimelightSubsystem;
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
      .withNetworkTableName("limelight-high")
      .withMountDepth(LimeLightConstants.HIGH_DISTANCE_FROM_FRONT)
      .withMountingHeight(LimeLightConstants.HIGH_MOUNT_HEIGHT)
      .withMountingAngle(LimeLightConstants.HIGH_MOUNT_ANGLE)
      .build()
  );
  private final LimelightSubsystem lowLimelightSubsystem = new LimelightSubsystem(LimelightConfig.Builder.create()
      .withNetworkTableName("limelight-low")
      .withMountDepth(LimeLightConstants.LOW_DISTANCE_FROM_FRONT)
      .withMountingHeight(LimeLightConstants.LOW_MOUNT_HEIGHT)
      .withMountingAngle(LimeLightConstants.LOW_MOUNT_ANGLE)
      .build()
  );
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final XboxController driverController = new XboxController(PORT_ID_DRIVER_CONTROLLER);
  private final XboxController operatorConsole = new XboxController(PORT_ID_OPERATOR_CONSOLE);

  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(driverController, driveTrainSubsystem);
  private final TeleOperateCommand teleOperateCommand = new TeleOperateCommand(operatorConsole, indexerSubsystem);
  private final IndexCommand indexCommand = new IndexCommand(indexerSubsystem);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();

    highLimelightSubsystem.setProfile(Profile.FAR);

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

    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whenHeld(new AimShooterCommand(highLimelightSubsystem, driveTrainSubsystem));

    new JoystickButton(operatorConsole, XboxController.Button.kX.value)
        .whenHeld(new RotateWheelCommand(controlPanelSubsystem));

    new JoystickButton(operatorConsole, XboxController.Button.kStart.value)
        .whenHeld(new SetColorCommand(controlPanelSubsystem));

    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whenPressed(teleDriveCommand::toggleSlowMode);
    
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whenPressed(teleDriveCommand::toggleReverseMode);

    new JoystickButton(driverController, XboxController.Button.kBumperLeft.value)
       .whenPressed(driveTrainSubsystem::saveCurrentPose);

    new JoystickButton(driverController, XboxController.Button.kBumperRight.value).whenPressed(() ->
      driveTrainSubsystem.createCommandForTrajectory(
          TrajectoryGenerator.generateTrajectory(
            driveTrainSubsystem.getCurrentPose(),
            Collections.emptyList(),
            driveTrainSubsystem.getSavedPose(),
            new TrajectoryConfig(MAX_SPEED_AUTO, MAX_ACCELERATION_AUTO)
                .setKinematics(DRIVE_KINEMATICS)
                .addConstraint(VOLTAGE_CONSTRAINT)))
      .schedule());

    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whenHeld(new ShootCommand(shooterSubsystem, indexerSubsystem, highLimelightSubsystem, lowLimelightSubsystem));

    new JoystickButton(operatorConsole, XboxController.Button.kA.value)
        .whenHeld(new InstantCommand(() -> indexerSubsystem.runManually(1.0), indexerSubsystem));

    new JoystickButton(operatorConsole, XboxController.Button.kB.value)
        .whenHeld(new InstantCommand(() -> indexerSubsystem.runManually(-1.0), indexerSubsystem));
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
    indexerSubsystem.setDefaultCommand(indexCommand);
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
