package frc.robot.subsystems;

/**
 * LimelightConfig
 */
public class LimelightConfig {

  private String networkTableName;

  private double mountHeight;

  private double mountDepth;

  private double mountAngle;

  public String getNetworkTableName() {
    return networkTableName;
  }

  public double getMountHeight() {
    return mountHeight;
  }

  public double getMountDepth() {
    return mountDepth;
  }

  public double getMountAngle() {
    return mountAngle;
  }

  private void setNetworkTableName(String networkTableName) {
    this.networkTableName = networkTableName;
  }

  private void setMountHeight(double mountHeight) {
    this.mountHeight = mountHeight;
  }

  private void setMountDepth(double mountDepth) {
    this.mountDepth = mountDepth;
  }

  private void setMountAngle(double mountAngle) {
    this.mountAngle = mountAngle;
  }

  public static class Builder {
    
    private LimelightConfig limelightConfig = new LimelightConfig();

    public static Builder create() {
      return new Builder();
    }

    public Builder withNetworkTableName(String networkTableName) {
      limelightConfig.setNetworkTableName(networkTableName);
      return this;
    }

    public Builder withMountingHeight(double mountingHeight) {
      limelightConfig.setMountHeight(mountingHeight);
      return this;
    }

    public Builder withMountDepth(double mountDepth) {
      limelightConfig.setMountDepth(mountDepth);
      return this;
    }

    public Builder withMountingAngle(double mountingAngle) {
      limelightConfig.setMountAngle(mountingAngle);
      return this;
    }

    public LimelightConfig build() {
      return limelightConfig;
    }

  }

}