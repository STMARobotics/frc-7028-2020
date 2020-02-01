/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
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

  public static final class DriveTrainConstants {
    public static final int DEVICE_ID_RIGHT_MASTER = 0;
    public static final int DEVICE_ID_RIGHT_SLAVE = 1;
    public static final int DEVICE_ID_LEFT_MASTER = 2;
    public static final int DEVICE_ID_LEFT_SLAVE = 3;

    public static final int SENSOR_UNITS_PER_ROTATION = 4096;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    public static final double TRACK_WIDTH_METERS = 0.555625;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double kS = 0.829;

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double kV = 3.04;

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double kA = 0.676;

    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double kP = 0.75;

    public static final double CLOSED_LOOP_RAMP = .2;
    public static final double OPEN_LOOP_RAMP = .25;
  }

  public static final class ControllerConstants {
    public static final int PORT_ID_DRIVER_CONTROLLER = 0;
    public static final int PORT_ID_OPERATOR_CONSOLE = 1;
  }

  public static final class TrajectoryConstants {

    // Max speed in meters per second
    public static final double MAX_SPEED_AUTO = 3;

    // Max acceleration in meters per second per second
    public static final double MAX_ACCELERATION_AUTO = 2;

    // Max voltage
    public static final double MAX_VOLTAGE_AUTO = 11;

    public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = 
        new DifferentialDriveVoltageConstraint(FEED_FORWARD, DRIVE_KINEMATICS, MAX_VOLTAGE_AUTO);

    // Baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

  public static final class DriverConstants {
    
    public static final double ROTATION_MULTIPLIER = .78;

    public static final double SLOW_MODE_SPEED_MULTIPLIER = .6;
    public static final double SLOW_MODE_ROTATION_MULTIPLIER = .9;

    public static final double DEADBAND_HIGH = 0.9;
    public static final double DEADBAND_LOW = -DEADBAND_HIGH;
  }

  public static final class ArcadeConstants {
    // Max speed to drive in teleop in meters per second
    public static final double MAX_SPEED_ARCADE = 3.0;

    // Max angular velocity in teleop in radians per second
    public static final double MAX_ANGULAR_VEL_ARCADE = Units.degreesToRadians(360);

    // Max rate of change for speed per second
    public static final double SPEED_RATE_LIMIT_ARCADE = 1.5;

    // Max rate of change for rotation per second
    public static final double ROTATE_RATE_LIMIT_ARCADE = 3.0;
  }

  public static final class AimConstants {

    public static final double kP = 0.50;
    public static final double kD = 0.0;

    public static final double RANGE_HIGH = 1;
    public static final double RANGE_LOW = -1;
  }

  public static final class LimeLightConstants {

    public static final double TARGET_X_MAX = 29.8;

    public static final double TARGET_ACQUIRED = 1.0;

  }

  public static final class ControlPanelConstants {

    public static final int SENSOR_UNITS_PER_ROTATION = 8192;

    public static final double kS = 2.09;
    public static final double kV = 0.109;
    public static final double kA = 0.172;

    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double kP_VELOCITY = 0.1;
    public static final double kP_POSITION = 1;
    public static final double kD_POSITION = 0;

    public static final double ROTATE_RPM = 5;
    public static final double SET_COLOR_RPM = 3;

  }

}
