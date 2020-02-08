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
import frc.robot.Dashboard;

/**
 * IndexerSubsystem
 */
public class IndexerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX belt = new WPI_TalonSRX(DEVICE_ID_BELT);

  private final DigitalInput intakeSensor = new DigitalInput(PORT_ID_INTAKE_SENSOR);
  private final DigitalInput spacerSensor = new DigitalInput(PORT_ID_SPACER_SENSOR);
  private final DigitalInput fullSensor = new DigitalInput(PORT_ID_FULL_SENSOR);

  private final ShuffleboardLayout dashboard = Dashboard.subsystemsTab.getLayout("Indexer", BuiltInLayouts.kList)
      .withSize(2, 3).withPosition(6, 0);
  private final ShuffleboardLayout detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
      .withProperties(Map.of("numberOfColumns", 2, "numberOfRows", 2));

  private boolean running;
  private int state = 0;

  public IndexerSubsystem() {
    dashboard.add(this);
    detailDashboard.addBoolean("Intake", () -> intakeSensor.get());
    detailDashboard.addBoolean("Spacer", () -> spacerSensor.get());
    detailDashboard.addBoolean("Full", () -> fullSensor.get());
    detailDashboard.addNumber("Balls", () -> state).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 5));
    belt.setInverted(true);
  }

  public void runManually(double speed) {
    belt.set(speed);
  }

  private void run() {
    belt.set(BELT_RUN_SPEED);
    running = true;
  }
  
  private void stop() {
    belt.set(0.0);
    running = false;
  }

  private void output() {
    
  }

  public void intake() {
    // sensors return false when something is detected
    if ((intakeSensor.get() && spacerSensor.get() && fullSensor.get()) ||
        (!fullSensor.get() || (spacerSensor.get() && intakeSensor.get()))) {
          if (running) {
            state++;
          } if (!fullSensor.get()) {
            state = 5;
          }
          stop();
    } else {
      run();
    }
  }

  public void shoot() {
    belt.set(1.0);
  }

  public void stopIndexer() {
    stop();
  }
  
}