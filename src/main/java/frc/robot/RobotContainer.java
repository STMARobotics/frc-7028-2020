/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.Auto.MAX_ACCELERATION_METERS_PER_SECOND;
import static frc.robot.Constants.Auto.MAX_SPEED_METERS_PER_SECOND;
import static frc.robot.Constants.Auto.VOLTAGE_CONSTRAINT;
import static frc.robot.Constants.Controller.PORT_ID_DRIVER_CONTROLLER;
import static frc.robot.Constants.Controller.PORT_ID_OPERATOR_CONSOLE;
import static frc.robot.Constants.DriveTrain.DRIVE_KINEMATICS;

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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

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

  private final XboxController driverController = new XboxController(PORT_ID_DRIVER_CONTROLLER);
  private final XboxController operatorConsole = new XboxController(PORT_ID_OPERATOR_CONSOLE);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();

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
    new JoystickButton(driverController, XboxController.Button.kBumperLeft.value)
        .whenPressed(driveTrainSubsystem::saveCurrentPose);
    new JoystickButton(driverController, XboxController.Button.kBumperRight.value).whenPressed(() ->
      new PrintCommand("Running path")
      .andThen(driveTrainSubsystem.createCommandForTrajectory(
          TrajectoryGenerator.generateTrajectory(
            driveTrainSubsystem.getCurrentPose(),
            Collections.emptyList(),
            driveTrainSubsystem.getSavedPose(),
            new TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND)
                .setKinematics(DRIVE_KINEMATICS)
                .addConstraint(VOLTAGE_CONSTRAINT))))
      .andThen(new PrintCommand("Done running path"))
      .schedule());
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(new TeleDriveCommand(driverController, driveTrainSubsystem));
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
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

  public void resetOdometry() {
    new InstantCommand(driveTrainSubsystem::resetOdometry, driveTrainSubsystem).schedule();
  }
}
