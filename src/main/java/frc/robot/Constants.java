/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveTrain {
    public static final int DEVICE_ID_LEFT_MASTER = 2;
    public static final int DEVICE_ID_LEFT_SLAVE = 0;
    public static final int DEVICE_ID_RIGHT_MASTER = 3;
    public static final int DEVICE_ID_RIGHT_SLAVE = 1;

    public static final int SENSOR_UNITS_PER_ROTATION = 4096;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    // TODO all of these have to be set for our bot
    public static final double TRACK_WIDTH_METERS = 0.555625;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = 
      new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction */
    public static final double STATIC_VOLTS = 0.22;

    /** Voltage needed to hold (or “cruise”) at a given constant velocity */
    public static final double VOLT_SECONDS_PER_METER = 1.98;

    /** Voltage needed to induce a given acceleration in the motor shaft */
    public static final double VOLT_SECONDS_SQUARED_PER_METER = 0.2;

    public static final double P_GAIN_DRIVE_VEL = 8.5;
  }

  public static final class Controller {
    public static final int PORT_ID_DRIVER_CONTROLLER = 0;
    public static final int PORT_ID_OPERATOR_CONSOLE = 1;
  }

  // TODO all of these have to be set for our bot
  public static final class Auto {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

}
