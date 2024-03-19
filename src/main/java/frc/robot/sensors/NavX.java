package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NavX extends SubsystemBase {
    private AHRS ahrs;
    private NetworkTableEntry yawEntry;
    private NetworkTableEntry pitchEntry;
    private NetworkTableEntry rollEntry;

    private NavX() {
        ahrs = new AHRS(SPI.Port.kMXP);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable navxTable = inst.getTable("NavX");
        yawEntry = navxTable.getEntry("yaw");
        pitchEntry = navxTable.getEntry("pitch");
        rollEntry = navxTable.getEntry("roll");
    }

    private static NavX instance = null;

    public static synchronized NavX getInstance() {
        if (instance == null) {
            instance = new NavX();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the NetworkTable entries with the latest navX data
        yawEntry.setDouble(ahrs.getYaw());
        pitchEntry.setDouble(ahrs.getPitch());
        rollEntry.setDouble(ahrs.getRoll());
    }

    /**
     * Resets the yaw of the navX to 0 degrees.
     */
    public void reset() {
        System.out.print("****** RESET ******");
        System.out.print("****** RESET ******");
        System.out.print("****** RESET ******");
        ahrs.reset();
    } 

    public Rotation2d getRotation2d() {
        return ahrs.getRotation2d();
    }
}