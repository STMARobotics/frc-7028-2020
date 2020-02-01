package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Dashboard;

/**
 * ControlPanelSubsystem
 */
public class ControlPanelSubsystem extends SubsystemBase {

  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  private final Color kBlueTarget = ColorMatch.makeColor(0.146, 0.446, 0.406);
  private final Color kGreenTarget = ColorMatch.makeColor(0.189, 0.562, 0.249);
  private final Color kRedTarget = ColorMatch.makeColor(0.452, 0.385, 0.163);
  private final Color kYellowTarget = ColorMatch.makeColor(0.309, 0.555, 0.135);
  private final Color kWhiteTarget = ColorMatch.makeColor(0.259, 0.488, 0.252);

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final WPI_TalonSRX motor = new WPI_TalonSRX(4);
  
  private final ShuffleboardLayout dashboard = Dashboard.subsystemsTab.getLayout("Control Panel", BuiltInLayouts.kGrid)
    .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 5)).withSize(2, 5).withPosition(2, 0);
  private final SuppliedValueWidget<Boolean> colorWidget = dashboard.addBoolean("Color", () -> true);
  private final ShuffleboardLayout colorGrid = dashboard.getLayout("Color Info", BuiltInLayouts.kGrid)
    .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));
  private final ShuffleboardLayout speedGrid = dashboard.getLayout("Speed", BuiltInLayouts.kGrid)
    .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 1));
    
  private String colorString;

  public ControlPanelSubsystem() {
    dashboard.add(this);
    dashboard.add("Motor", motor);
    speedGrid.addNumber("Raw", motor::getSelectedSensorVelocity);
    speedGrid.addNumber("RPM", () -> stepsPerDecisecToRPS(motor.getSelectedSensorVelocity()) * 60);
    colorGrid.addNumber("Color Red", () -> colorSensor.getColor().red);
    colorGrid.addNumber("Color Green", () -> colorSensor.getColor().green);
    colorGrid.addNumber("Color Blue", () -> colorSensor.getColor().blue);
    colorGrid.addNumber("Color confidence", () -> m_colorMatcher.matchClosestColor(colorSensor.getColor()).confidence);

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    talonConfig.slot0.kP = ControlPanelConstants.kP_VELOCITY;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot1.kP = ControlPanelConstants.kP_POSITION;
    talonConfig.slot1.kI = 0.0;
    talonConfig.slot1.kD = ControlPanelConstants.kD_POSITION;
    motor.setNeutralMode(NeutralMode.Brake);
    stopWheel();

    motor.configAllSettings(talonConfig);
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    motor.setSensorPhase(true);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kWhiteTarget);
  }

  @Override
  public void periodic() {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kWhiteTarget) {
      colorString = "White";
    } else if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "White";
    }

    colorWidget.withProperties(Map.of("colorWhenTrue", colorString));
  }

  public String getColor() {
    return colorString;
  }

  public void spinForRotations() {
    spinSpeed(ControlPanelConstants.ROTATE_RPM);
  }

  public void spinForColor() {
    spinSpeed(ControlPanelConstants.SET_COLOR_RPM);
  }

  public void spinSpeed(double rpm) {
    var accel = (rpm - stepsPerDecisecToRPS(motor.getSelectedSensorVelocity())) / .20;
    var leftFeedForwardVolts = ControlPanelConstants.FEED_FORWARD.calculate(rpm, accel);
    motor.selectProfileSlot(0, 0);
    motor.set(
        ControlMode.Velocity,
        rpmToStepsPerDecisec(ControlPanelConstants.SET_COLOR_RPM),
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
  }

  public void stopWheel() {
    motor.set(0);
  }

  public void stopHere() {
    motor.selectProfileSlot(1, 1);
    motor.set(ControlMode.Position, motor.getSelectedSensorPosition());
  }

  public static double stepsToRotations(int steps) {
    return steps / (double) ControlPanelConstants.SENSOR_UNITS_PER_ROTATION;
  }

  public static double stepsPerDecisecToRPS(int stepsPerDecisec) {
    return stepsToRotations(stepsPerDecisec) * 10;
  }

  public static double rotationsToSteps(double rotations) {
    return rotations * ControlPanelConstants.SENSOR_UNITS_PER_ROTATION;
  }

  public static double rpmToStepsPerDecisec(double rpm) {
    return rotationsToSteps(rpm) / 600.0;
  }

}
