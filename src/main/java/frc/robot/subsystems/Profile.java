package frc.robot.subsystems;

/**
 * Profiles
 */
public enum Profile {

  NEAR(0.0),
  FAR(1.0);

  public final double pipelineId;

  private Profile(double pipelineId) {
    this.pipelineId = pipelineId;
  }

}