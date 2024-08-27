package frc.robot.libs;

public class SwerveDriveConfig {
    // Swerve Drive Configuration
    public double wheelsWidth; // meters
    public double wheelsLength; // meters
    public SwerveModuleConfig[] modules;

    public SwerveDriveConfig(double wheelsWidth, double wheelsLength, SwerveModuleConfig[] modules) {
        this.wheelsWidth = wheelsWidth;
        this.wheelsLength = wheelsLength;
        this.modules = modules;
    }
}
