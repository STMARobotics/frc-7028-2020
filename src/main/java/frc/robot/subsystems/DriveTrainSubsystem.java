package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveTrain.*;

/**
 * DriveTrainSubsystem
 */
public class DriveTrainSubsystem extends SubsystemBase {

    private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DEVICE_ID_LEFT_MASTER);
    private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(DEVICE_ID_LEFT_SLAVE);
    private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(DEVICE_ID_RIGHT_MASTER);
    private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(DEVICE_ID_RIGHT_SLAVE);
    private final DifferentialDrive differentialDrive = new DifferentialDrive(
        new SpeedControllerGroup(leftMaster, leftSlave),
        new SpeedControllerGroup(rightMaster, rightSlave));

    public DriveTrainSubsystem() {

    }

    /**
     * Drives the robot by adjusting x axis speed and z axis rotation
     * @param speed speed along the x axis [-1.0..1.0]
     * @param rotation rotation rate along the z axis [-1.0..1.0]
     */
    public void arcadeDrive(double speed, double rotation) {
        arcadeDrive(speed, rotation, false);
    }

    /**
     * Drives the robot by adjusting x axis speed and z axis rotation
     * @param speed speed along the x axis [-1.0..1.0]
     * @param rotation rotation rate along the z axis [-1.0..1.0]
     * @param useSquares if set, decreases input sensitivity at low speeds
     */
    public void arcadeDrive(double speed, double rotation, boolean useSquares) {
        differentialDrive.arcadeDrive(speed, rotation, useSquares);
    }

    /**
     * Drives the robot by individually addressing the left and right side of the drive train
     * @param leftSpeed speed of the left motors [-1.0..1.0]
     * @param rightSpeed speed of the right motors [-1.0..1.0]
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        tankDrive(leftSpeed, rightSpeed, false);
    }

    /**
     * Drives the robot by individually addressing the left and right side of the drive train
     * @param leftSpeed speed of the left motors [-1.0..1.0]
     * @param rightSpeed speed of the right motors [-1.0..1.0]
     * @param useSquares if set, decreases input sensitivity at low speeds
     */
    public void tankDrive(double leftSpeed, double rightSpeed, boolean useSquares) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed, useSquares);
    }

    /**
     * Sets the neutral mode for the drive train
     * @param neutralMode the desired neutral mode
     */
    public void setNeutralMode(NeutralMode neutralMode) {
        leftMaster.setNeutralMode(neutralMode);
        leftSlave.setNeutralMode(neutralMode);
        rightMaster.setNeutralMode(neutralMode);
        rightSlave.setNeutralMode(neutralMode);
    }

    /**
     * Converts inches to wheel revolutions
     * @param inches inches
     * @return wheel revolutions
     */
    public static double insToRevs(double inches) {
        return inches / WHEEL_CIRCUMFERENCE_INCHES;
    }

    /**
     * Converts inches to encoder steps
     * @param inches inches
     * @return encoder steps
     */
    public static double insToSteps(double inches) {
        return (insToRevs(inches) * SENSOR_UNITS_PER_ROTATION);
    }

    /**
     * Converts inches per second to encoder steps per decisecond
     * @param inchesPerSec inches per second
     * @return encoder steps per decisecond (100 ms)
     */
    public static double insPerSecToStepsPerDecisec(double inchesPerSec) {
        return insToSteps(inchesPerSec) * .1;
    }
    
}