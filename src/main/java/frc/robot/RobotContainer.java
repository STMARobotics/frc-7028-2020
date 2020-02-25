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
import java.util.ArrayList;
import java.util.Map;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.InstantWhenDisabledCommand;
import frc.robot.commands.PIDPixyAssistCommand;
import frc.robot.commands.RotateWheelCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.SetColorCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleDriveCommand;
import frc.robot.commands.TeleOperateCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
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
import frc.robot.testMode.TestIntakeCommand;
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
  private final ControlPanelSubsystem controlPanelSubsystem = new ControlPanelSubsystem();

  private final XboxController driverController = new XboxController(PORT_ID_DRIVER_CONTROLLER);
  private final XboxController operatorConsole = new XboxController(PORT_ID_OPERATOR_CONSOLE);

  private final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(driverController, driveTrainSubsystem);
  private final TeleOperateCommand teleOperateCommand = new TeleOperateCommand(operatorConsole, indexerSubsystem);
  private final IndexCommand indexCommand = new IndexCommand(indexerSubsystem);
  private final ShootCommand shootCommand = new ShootCommand(Integer.MAX_VALUE, shooterSubsystem, indexerSubsystem, 
    highLimelightSubsystem, lowLimelightSubsystem, driveTrainSubsystem);

  private final AutoGenerator autoGenerator = new AutoGenerator(driveTrainSubsystem, highLimelightSubsystem,
    lowLimelightSubsystem, indexerSubsystem, intakeSubsystem, shooterSubsystem, pixyVision, controlPanelSubsystem);

  private final UsbCamera camera;

  public RobotContainer() {
    camera = CameraServer.getInstance().startAutomaticCapture();
    try {
      camera.setFPS(15);
      camera.setResolution(320, 240);
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure driver camera", true);
    }

    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();
    
    autoGenerator.configureAutonomous();

    configureSubsystemDashboard();
    // configureCommandDashboard();
    configureDriverDashboard();
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
        .whenHeld(new ConditionalCommand(
            new RumbleCommand(driverController, RumbleType.kLeftRumble),
            new RunIntakeCommand(intakeSubsystem),
            indexerSubsystem::isFull));

    var pixyHeldCommand = new PIDPixyAssistCommand(driveTrainSubsystem, pixyVision)
        .andThen(
          new RunCommand(() -> driveTrainSubsystem.arcadeDrive(.2, 0, false), driveTrainSubsystem).withTimeout(0.25))
        .andThen(new InstantCommand(driveTrainSubsystem::stop, driveTrainSubsystem))
        .deadlineWith(new RunCommand(intakeSubsystem::intake, intakeSubsystem))
        .andThen(new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem));
    
    var pixyReleaseCommand = new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem)
        .andThen(new InstantCommand(driveTrainSubsystem::stop, driveTrainSubsystem));

    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileHeld(new ConditionalCommand(
            new RumbleCommand(driverController, RumbleType.kLeftRumble),
            pixyHeldCommand,
            indexerSubsystem::isFull))
        .whenReleased(pixyReleaseCommand);

    new POVButton(driverController, 0)
        .whenPressed(new TurnToAngleCommand(0, driveTrainSubsystem));

    new POVButton(driverController, 90)
        .whenPressed(new TurnToAngleCommand(90, driveTrainSubsystem));

    new POVButton(driverController, 180)
        .whenPressed(new TurnToAngleCommand(180, driveTrainSubsystem));

    new POVButton(driverController, 270)
        .whenPressed(new TurnToAngleCommand(-90, driveTrainSubsystem));

    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .toggleWhenPressed(new StartEndCommand(() -> {
            highLimelightSubsystem.enable();
            lowLimelightSubsystem.enable();
          }, () -> {
            highLimelightSubsystem.disable();
            lowLimelightSubsystem.disable();
          }));

    // Operator
    new JoystickButton(operatorConsole, OperatorConsoleButton.RightThumbButton.value)
        .whenPressed(makeLimelightProfileCommand(Profile.NEAR));
    
    new JoystickButton(operatorConsole, OperatorConsoleButton.LeftThumbButton.value)
        .whenPressed(makeLimelightProfileCommand(Profile.FAR));

    new JoystickButton(operatorConsole, OperatorConsoleButton.MiddleFingerButton.value)
        .whenHeld(new SetColorCommand(controlPanelSubsystem));
    
    new JoystickButton(operatorConsole, OperatorConsoleButton.IndexFingerButton.value)
        .whenHeld(new RotateWheelCommand(controlPanelSubsystem));
    
    new JoystickButton(operatorConsole, OperatorConsoleButton.PinkyFingerButton.value)
        .whenPressed(new RunCommand(controlPanelSubsystem::raiseArmPeriodic, controlPanelSubsystem));

    new JoystickButton(operatorConsole, OperatorConsoleButton.RingFingerButton.value)
        .whenPressed(new RunCommand(controlPanelSubsystem::lowerArmPeriodic, controlPanelSubsystem));
    
  }

  private void configureSubsystemCommands() {
    driveTrainSubsystem.setDefaultCommand(teleDriveCommand);
    indexerSubsystem.setDefaultCommand(indexCommand);
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
    // Auto chooser
    autoGenerator.addDashboardWidgets(Dashboard.driverTab);

    // Indexer
    var indexerLayout = Dashboard.driverTab.getLayout("Indexer", BuiltInLayouts.kGrid)
        .withSize(2, 1).withPosition(0, 1)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 1));
    indexerLayout.addBoolean("Full", indexerSubsystem::isFull);
    indexerLayout.addBoolean("Running", indexerSubsystem::isRunning);    
    indexerLayout.addNumber("Balls", indexerSubsystem::getBallCount).withWidget(BuiltInWidgets.kDial)
        .withSize(2, 1).withProperties(Map.of("min", 0, "max", 5));

    // Cameras
    Dashboard.driverTab.addString("Pipeline", () -> highLimelightSubsystem.getProfile().toString()).withPosition(6, 0);
    Dashboard.driverTab.add(camera).withSize(4, 3).withPosition(2, 0);
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
    return autoGenerator.getAutonomousCommand();
  }

  public void resetOdometry() {
    new InstantCommand(driveTrainSubsystem::resetOdometry, driveTrainSubsystem).schedule();
  }

  public Command[] getTestModeCommands() {
    var commands = new ArrayList<Command>();

    var testDrivetrainEntry = Dashboard.testModeTab.addPersistent("Test Drivetrain", false)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    
    var testDrivetrain = testDrivetrainEntry.getBoolean(false);

    if (testDrivetrain) {
      //add encoder commands, chain forward/reverse with a 1 second wait in between to allow the drivetrain to stop
      commands.add(new TestEncoderCommand(.25, driveTrainSubsystem).withTimeout(5)
        .andThen(new WaitCommand(1))
        //add a 5 second reverse test
        .andThen(new TestEncoderCommand(-.25, driveTrainSubsystem).withTimeout(5)));
    }

    // add intake encoder commands, run each direction with 1 second in between
    commands.add(new TestIntakeCommand(true, intakeSubsystem).withTimeout(5)
      .andThen(new WaitCommand(1))
      .andThen(new TestIntakeCommand(false, intakeSubsystem)).withTimeout(5));

    //add commands for each limelight system
    commands.add(new TestLimelightCommand(highLimelightSubsystem).withTimeout(10));
    commands.add(new TestLimelightCommand(lowLimelightSubsystem).withTimeout(10));

    commands.add(new TestIndexerCommand(indexerSubsystem).withTimeout(60));

    Command[] arr = new Command[commands.size()];
    return commands.toArray(arr);
  }
}
