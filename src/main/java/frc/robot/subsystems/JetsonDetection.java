package frc.robot.subsystems;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.UpperCamelCaseStrategy;
import com.fasterxml.jackson.databind.annotation.JsonNaming;

import lombok.Getter;

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

