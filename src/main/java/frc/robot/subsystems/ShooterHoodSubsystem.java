package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ShooterHoodSubsystem
 */
public class ShooterHoodSubsystem extends SubsystemBase{
  private final Servo hood = new Servo(0);
  
  public void setHoodValue(double hoodValue){
    System.out.print("updating hood value " + hoodValue);
    hood.set(hoodValue);
  }
  
  public double getHoodValue() {
    return hood.get();
  }
}