package frc.robot;

/**
 * This interface contains constants related to the robot's configuration and physical properties.
 */
public interface Constants {
    public static final double gearRatio = 6.12;
    public static final double botMass = 24.4;
    public static final double wheelDiameter = .1016;
    public static final double botLength = .3302;
    public static final double botWidth = .2794;
    public static final double maxSpeed = 4.0/*4.05968*/; // In meters per second, determined from the free speed of the bot via SwerveDriveSpecialties
    public static final double maxTurnSpeed = Double.MAX_VALUE; //These are basically infinite for our purposes 
    public static final double maxAcceleration = 4000;
    public static final double botRadius = Math.hypot(botLength, botWidth);
    public static final double maxChassisTurnSpeed = maxSpeed/botRadius; // Max Speed divided by the circumference a circle determined by the distance of the module from the center, divided by 2 pi to convert to radians
    public static final double encoderRotationToMeters = 2*Math.PI*((wheelDiameter/2)/gearRatio)/42;
    
    // Use these limits to restrict the current draw of the bot
    public static final int maxTurnAmps = 15;
    public static final int maxDriveAmps = 35;

    public static final double fieldLength = 286;
}


