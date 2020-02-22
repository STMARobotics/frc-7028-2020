package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.BELT_RUN_SPEED;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_BELT;
import static frc.robot.Constants.IndexerConstants.PORT_ID_FULL_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_INTAKE_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_SPACER_SENSOR;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * IndexerSubsystem
 */
public class IndexerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX belt = new WPI_TalonSRX(DEVICE_ID_BELT);

  private final DigitalInput intakeSensor = new DigitalInput(PORT_ID_INTAKE_SENSOR);
  private final DigitalInput spacerSensor = new DigitalInput(PORT_ID_SPACER_SENSOR);
  private final DigitalInput fullSensor = new DigitalInput(PORT_ID_FULL_SENSOR);

  private boolean running;
  private boolean shooting;
  private int ballCount = 0;

  public IndexerSubsystem() {
    belt.configFactoryDefault();
    belt.setInverted(true);
    new Trigger(() -> this.fullSensor.get()).whenActive(() -> ballCount = MathUtil.clamp(ballCount - 1, 0, 5));
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));
    dashboard.addNumber("Balls", () -> ballCount).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 5));
    detailDashboard.addBoolean("Intake", () -> intakeSensor.get());
    detailDashboard.addBoolean("Spacer", () -> spacerSensor.get());
    detailDashboard.addBoolean("Full", () -> fullSensor.get());
  }

  public void runManually(double speed) {
    belt.set(speed);
    shooting = false;
  }

  private void run() {
    belt.set(BELT_RUN_SPEED);
    shooting = false;
    running = true;
  }
  
  private void stop() {
    belt.set(0.0);
    shooting = false;
    running = false;
  }

  public void reverse() {
    belt.set(-1.0);
    shooting = false;
  }

  public void intake() {
    // sensors return false when something is detected
    if ((intakeSensor.get() && spacerSensor.get() && fullSensor.get()) ||
        (!fullSensor.get() || (spacerSensor.get() && intakeSensor.get()))) {
      if (running) {
        ballCount = MathUtil.clamp(ballCount + 1, 0, 5);
      }
      stop();
    } else {
      run();
    }
  }

  public void prepareToShoot() {
    if (fullSensor.get()) {
      belt.set(BELT_RUN_SPEED);
    } else {
      belt.set(0.0);
    }
  }

  public void shoot() {
    belt.set(1.0);
    shooting = true;
  }

  public boolean isReadyForBall() {
    return (intakeSensor.get() && spacerSensor.get() && fullSensor.get())
        || (shooting && intakeSensor.get() && spacerSensor.get());
  }

  public void stopIndexer() {
    stop();
  }

  public boolean isFull() {
    return !fullSensor.get();
  }

  public boolean isRunning() {
    return belt.get() != 0;
  }

  public int getBallCount() {
    return ballCount;
  }

  public void resetBallCount(int ballCount) {
    this.ballCount = ballCount;
  }
  
}