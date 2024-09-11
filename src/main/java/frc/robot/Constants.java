package frc.robot;

import frc.robot.libs.SwerveDriveConfig;
import frc.robot.libs.SwerveModuleConfig;

/**
 * This interface contains constants related to the robot's configuration and
 * physical properties.
 */
public interface Constants {
    // circle determined by the distance of the
    public static final double wheelsLength = 0.65; // meters - 25.5 inches
    public static final double wheelsWidth = 0.55; // meters - 21.5 inches

    SwerveModuleConfig[] modules = {
        new SwerveModuleConfig("FrontLeft", 13, 4, 3, -142, wheelsLength/2.0, wheelsWidth/2.0),
        new SwerveModuleConfig("FrontRight", 10, 8, 7, 25+15, wheelsLength/2.0, -wheelsWidth/2.0),
        new SwerveModuleConfig("BackLeft", 12, 2, 1, -139-18, -wheelsLength/2.0, wheelsWidth/2.0),
        new SwerveModuleConfig("BackRight", 11, 6, 5, 50, -wheelsLength/2.0, -wheelsWidth/2.0)};

    SwerveDriveConfig swerve_config = new SwerveDriveConfig(wheelsWidth, wheelsLength, modules);

    public static final double botRadius = 0.86;
    public static final double wheelDiameter = .1016;
    public static final double gearRatio = 6.12;
    public static final double encoderRotationToMeters = 2 * Math.PI * ((wheelDiameter / 2) / gearRatio) / 42;

    public static final double maxSpeed = 4.0; // In meters per second, determined from the free speed of the motors
    public static final double maxTurnSpeed = 1.3 * maxSpeed / botRadius; // Max Speed divided by the circumference a circle with the radius of the bot

    // Use these limits to restrict the current draw of the bot
    public static final int maxTurnAmps = 15;
    public static final int maxDriveAmps = 35;

    public static final double fieldLength = 286;
}
