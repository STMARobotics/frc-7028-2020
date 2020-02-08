package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/**
 * IndexerSubsystem
 */
public class IndexerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX indexerMotor = new WPI_TalonSRX(IndexerConstants.DEVICE_ID_INDEXER);
  //indexer motor
  //sensors

  public void runManually(double percentOut) {
    indexerMotor.set(percentOut);
    indexerMotor.setInverted(true);
  }

  public void intake() {
    
  }

  public void shoot() {
    indexerMotor.set(1.0);
  }

  public void stopIndexer() {
    indexerMotor.set(0.0);
  }
  
}