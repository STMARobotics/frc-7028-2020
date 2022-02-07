package frc.robot.subsystems;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.UpperCamelCaseStrategy;
import com.fasterxml.jackson.databind.annotation.JsonNaming;

import lombok.Getter;

/**
 * This class holds the data sent by the Jetson. The Jetson sends one big JSON string of data for a detection instead
 * of sending each value in a separate network table entry so we are can be certain to get all matching values for a
 * frame.
 */
@Getter
@JsonNaming(UpperCamelCaseStrategy.class)
public class JetsonDetection {
  private String classLabel;
  private String classID;
  private String instanceID;
  private double area;
  private double bottom;
  private double centerX;
  private double centerY;
  private double confidence;
  private double height;
  private double left;
  private double right;
  private double top;
  private double width;
  private double timestamp;
  private double targetX;
  private double targetY;
  private double targetDistance;
}

