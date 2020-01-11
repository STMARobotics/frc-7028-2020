/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.Controller.PORT_ID_DRIVER_CONTROLLER;
import static frc.robot.Constants.Controller.PORT_ID_OPERATOR_CONSOLE;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();

  private final XboxController driverController = new XboxController(PORT_ID_DRIVER_CONTROLLER);
  private final XboxController operatorConsole = new XboxController(PORT_ID_OPERATOR_CONSOLE);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(driverController, driveTrainSubsystem);

  private final DifferentialDriveOdometry differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyroSubsystem.getGyroPosition()));

  public RobotContainer() {
    // Configure the button bindings

    // SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();
    configureSubsystemCommands();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(new TeleDriveCommand(driverController, driveTrainSubsystem));
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
  
  public void updateOdometry() {
    differentialDriveOdometry.update(Rotation2d.fromDegrees(gyroSubsystem.getGyroPosition()), driveTrainSubsystem.getLeftEncoderPosition(), driveTrainSubsystem.getRightEncoderPosition());
  }

  public void zeroDriveTrainEncoders() {
    driveTrainSubsystem.zeroDriveTrainEncoders();
  }

  public void printOdometry() {
    System.out.println(differentialDriveOdometry.getPoseMeters().getTranslation().div(4096));
  }

  public void printGyroPosition() {
    System.out.println(gyroSubsystem.getGyroPosition());
  }

  public void zeroGyroPosition() {
    gyroSubsystem.reset();
  }

  public void updateEncoderPositions() {
    SmartDashboard.putNumber("LeftEncoder", driveTrainSubsystem.getLeftEncoderPosition());
    SmartDashboard.putNumber("RightEncoder", driveTrainSubsystem.getRightEncoderPosition());
  }
}
