package frc.robot.subsystems;

import static com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic;
import static com.ctre.phoenix.motorcontrol.DemandType.ArbitraryFeedForward;
import static frc.robot.Constants.ControlPanelConstants.ARM_DOWN_POSITION;
import static frc.robot.Constants.ControlPanelConstants.ARM_EDGES_PER_DEGREE;
import static frc.robot.Constants.ControlPanelConstants.ARM_GRAVITY_FEED_FORWARD;
import static frc.robot.Constants.ControlPanelConstants.ARM_HORIZONTAL_POSITION;
import static frc.robot.Constants.ControlPanelConstants.ARM_UP_POSITION;

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

  private final WPI_TalonSRX spinnerMotor = new WPI_TalonSRX(ControlPanelConstants.DEVICE_ID_CONTROL_PANEL);
  private final WPI_TalonSRX armMotor = new WPI_TalonSRX(ControlPanelConstants.DEVICE_ID_CONTROL_PANEL_ARM);

  private SuppliedValueWidget<Boolean> colorWidget;

  private String colorString;

  public ControlPanelSubsystem() {

    TalonSRXConfiguration spinnerConfig = new TalonSRXConfiguration();
    spinnerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    spinnerConfig.slot0.kP = ControlPanelConstants.kP_VELOCITY;
    spinnerConfig.slot0.kI = 0.0;
    spinnerConfig.slot0.kD = 0.0;
    spinnerConfig.slot1.kP = ControlPanelConstants.kP_POSITION;
    spinnerConfig.slot1.kI = 0.0;
    spinnerConfig.slot1.kD = ControlPanelConstants.kD_POSITION;

    spinnerMotor.configAllSettings(spinnerConfig);
    spinnerMotor.setNeutralMode(NeutralMode.Brake);
    spinnerMotor.overrideLimitSwitchesEnable(false);
    spinnerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    spinnerMotor.setSensorPhase(true);

    TalonSRXConfiguration armConfig = new TalonSRXConfiguration();
    armConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    armConfig.slot0.kP = ControlPanelConstants.kP_ARM;
    armConfig.slot0.kI = 0.0;
    armConfig.slot0.kD = 0.0;
    armConfig.clearPositionOnLimitR = true;

    armMotor.configAllSettings(armConfig);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.setSensorPhase(true);
    armMotor.configMotionSCurveStrength(ControlPanelConstants.ARM_S_CURVE_STRENGTH);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kWhiteTarget);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    colorWidget = dashboard.addBoolean("Color", () -> true);
    var colorGrid = dashboard.getLayout("Color Info", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));
    colorGrid.addNumber("Color Red", () -> colorSensor.getColor().red);
    colorGrid.addNumber("Color Green", () -> colorSensor.getColor().green);
    colorGrid.addNumber("Color Blue", () -> colorSensor.getColor().blue);
    colorGrid.addNumber("Color confidence", () -> m_colorMatcher.matchClosestColor(colorSensor.getColor()).confidence);

    var speedGrid = dashboard.getLayout("Speed", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 1));
    speedGrid.addNumber("Raw Velocity", spinnerMotor::getSelectedSensorVelocity);
    speedGrid.addNumber("RPM", () -> stepsPerDecisecToRPS(spinnerMotor.getSelectedSensorVelocity()) * 60);
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

    if (colorWidget != null) {
      colorWidget.withProperties(Map.of("colorWhenTrue", colorString));
    }
  }

  public String getColor() {
    return colorString;
  }

  public void spinForRotations() {
    spinSpeed(ControlPanelConstants.ROTATE_RPM / 60);
  }

  public void spinForColor() {
    spinSpeed(ControlPanelConstants.SET_COLOR_RPM / 60);
  }

  public void spinSpeed(double rpm) {
    var accel = (rpm - stepsPerDecisecToRPS(spinnerMotor.getSelectedSensorVelocity())) / .20;
    var leftFeedForwardVolts = ControlPanelConstants.FEED_FORWARD.calculate(rpm, accel);
    spinnerMotor.selectProfileSlot(0, 0);
    spinnerMotor.set(ControlMode.Velocity, rpmToStepsPerDecisec(ControlPanelConstants.SET_COLOR_RPM),
        DemandType.ArbitraryFeedForward, leftFeedForwardVolts / 12);
  }

  public void stopWheel() {
    spinnerMotor.set(0);
  }

  public void stopHere() {
    spinnerMotor.selectProfileSlot(1, 1);
    spinnerMotor.set(ControlMode.Position, spinnerMotor.getSelectedSensorPosition());
  }

  public boolean isArmUp() {
    return armMotor.isRevLimitSwitchClosed() == 1;
  }

  public boolean isArmDown() {
    return armMotor.isFwdLimitSwitchClosed() == 1;
  }

  /**
   * Moves the arm to the up position when the position is unknown
   */
  public void calibrateArm() {
    if (!isArmUp()) {
      armMotor.set(ControlMode.Velocity, -200);
    }
  }

  private double calculateArbFeedForward() {
    // Arm feedforward to account for gravity.
    // See
    // https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#gravity-offset-arm
    int currentPos = armMotor.getSelectedSensorPosition();
    double degrees = (currentPos - ARM_HORIZONTAL_POSITION) / ARM_EDGES_PER_DEGREE;
    double cosineScalar = Math.cos(Math.toRadians(degrees));
    return ARM_GRAVITY_FEED_FORWARD * cosineScalar;
  }

  /**
   * Call periodically to raise the arm. Use {@link #isArmUp()} to tell when the
   * arm is up.
   */
  public void raiseArmPeriodic() {
    if (!isArmUp()) {
      armMotor.set(MotionMagic, ARM_UP_POSITION, ArbitraryFeedForward, calculateArbFeedForward());
    }
  }

  /**
   * Call periodically to lower the arm. Use {@link #isArmUp()} to tell when the arm is up.
   */
  public void lowerArmPeriodic() {
    if (!isArmDown()) {
      armMotor.set(MotionMagic, ARM_DOWN_POSITION, ArbitraryFeedForward, calculateArbFeedForward());
    }
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
