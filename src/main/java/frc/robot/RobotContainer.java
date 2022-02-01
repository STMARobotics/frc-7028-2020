/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.ControllerConstants.PORT_ID_DRIVER_CONTROLLER;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.JetsonBallCommand;
import frc.robot.commands.LimelightBallCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.JetsonSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

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
  private final LimelightSubsystem intakeLimelightSubsystem = new LimelightSubsystem(LimeLightConstants.LOW_NAME);
  private final JetsonSubsystem jetsonSubsystem = new JetsonSubsystem();

  private final XboxController driverController = new XboxController(PORT_ID_DRIVER_CONTROLLER);

  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(driverController, driveTrainSubsystem);

  private final UsbCamera camera;

  private SendableChooser<String> cargoColorChooser = new SendableChooser<>();

  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    camera = CameraServer.startAutomaticCapture();
    try {
      camera.setFPS(15);
      camera.setResolution(160, 120);
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure driver camera", true);
    }

    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();
    
    configureSubsystemDashboard();
    configureDriverDashboard();

    cargoColorChooser.setDefaultOption("Both", "Both");
    cargoColorChooser.addOption("RedCargo", "RedCargo");
    cargoColorChooser.addOption("BlueCargo", "BlueCargo");
    SmartDashboard.putData("Cargo Color", cargoColorChooser);
    NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Cargo Color")
      .addEntryListener("selected", new TableEntryListener() {
        @Override
        public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
            int flags) {
          jetsonSubsystem.setCargoColor(value.getString());
        }
      }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whenPressed(teleDriveCommand::toggleSlowMode);

    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whenPressed(teleDriveCommand::toggleReverseMode);

    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileHeld(new JetsonBallCommand(driveTrainSubsystem, jetsonSubsystem).perpetually());

    var limelightIntakeHeld = new LimelightBallCommand(driveTrainSubsystem, intakeLimelightSubsystem).perpetually();
    
    var limelightIntakeReleased = new InstantCommand(intakeLimelightSubsystem::disable, intakeLimelightSubsystem);

    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileHeld(limelightIntakeHeld)
        .whenReleased(limelightIntakeReleased);
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
  }

  private void configureSubsystemDashboard() {
    var drivetrainLayout = Dashboard.subsystemsTab.getLayout("Drivetrain", BuiltInLayouts.kList)
        .withSize(2, 5).withPosition(0, 0);
    driveTrainSubsystem.addDashboardWidgets(drivetrainLayout);
    drivetrainLayout.add(driveTrainSubsystem);

    var ballLimelightLayout = Dashboard.limelightsTab.getLayout("Ball Limelight", BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(2, 0);
    intakeLimelightSubsystem.addDashboardWidgets(ballLimelightLayout);
    ballLimelightLayout.add(intakeLimelightSubsystem);

  }

  private void configureDriverDashboard() {
    // Cameras
    Dashboard.driverTab.add(camera).withSize(4, 3).withPosition(3, 0);
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
    return null;
  }

  public void resetOdometry() {
    new InstantCommand(driveTrainSubsystem::resetOdometry, driveTrainSubsystem).schedule();
  }

}
