package frc.robot.libs;

/**
 * This class contains the configuration for a swerve module.
 */
public class SwerveModuleConfig {
    public String label;
    public int cancoderId;
    public int turnMotorCanId;
    public int driveMotorCanId;
    public double turnEncoderAngleOffset;
    public double xLocationOffset;
    public double yLocationOffset;

    /**
     * Constructs a SwerveModuleConfig object with the specified parameters.
     *
     * @param label the label of the swerve module ie. "FrontLeft"
     * @param cancoderId the ID of the CANCoder
     * @param turnMotorCanId the CAN ID of the turn motor
     * @param driveMotorCanId the CAN ID of the drive motor
     * @param turnEncoderAngleOffset the angle offset of the turn encoder
     * @param xLocationOffset the X location offset of the swerve module; positive is forward
     * @param yLocationOffset the Y location offset of the swerve module; positive is to the left
     */
    public SwerveModuleConfig(String label, int cancoderId, int turnMotorCanId, int driveMotorCanId, double turnEncoderAngleOffset, double xLocationOffset, double yLocationOffset) {
        this.label = label;
        this.cancoderId = cancoderId;
        this.turnMotorCanId = turnMotorCanId;
        this.driveMotorCanId = driveMotorCanId;
        this.turnEncoderAngleOffset = turnEncoderAngleOffset;
        this.xLocationOffset = xLocationOffset;
        this.yLocationOffset = yLocationOffset;
    }
}
