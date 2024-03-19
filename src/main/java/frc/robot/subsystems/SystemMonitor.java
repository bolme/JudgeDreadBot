package frc.robot.subsystems;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import java.util.HashMap;
import java.util.Map;

public class SystemMonitor extends SubsystemBase {
    private static SystemMonitor instance = null;
    private Map<String, MotorStats> motorStats;
    private Map<String, CANSparkMax> motors;
    private final NetworkTable motorMonitorTable;
    private final NetworkTable canBusTable;

    private boolean[] pdpActiveChannels = null;
    private double[] pdpRunningAverageAmps = null;
    private double[] pdpMaxAmps = null;
    private double[] pdpTotalEnergy = null;

    private PowerDistribution pdp;

    private double lastPdpUpdateTime = 0;

    private static final double HIGH_CURRENT_THRESHOLD = 30.0; // Adjust as needed
    private static final double HIGH_TEMP_THRESHOLD = 80.0; // Adjust as needed

    class MotorStats {
        private double maxCurrent = 0.0;
        private double maxSpeed = 0.0;
        private double totalRevolutions = 0.0;
        private double currentPosition = 0.0;
        private double currentSpeed = 0.0;
        private double lastPosition = 0.0;


        public void updateMaxCurrent(double current) {
            if (current > maxCurrent) {
                maxCurrent = current;
            }
        }

        public void updateMaxSpeed(double speed) {
            if (speed > maxSpeed) {
                maxSpeed = speed;
            }
        }

        public void updateTotalRevolutions(double position) {
            totalRevolutions += Math.abs(position - lastPosition);
            lastPosition = position;
        }

        public void updateCurrentPosition(double position) {
            currentPosition = position;
        }

        public void updateCurrentSpeed(double speed) {
            currentSpeed = speed;
        }

        public double getMaxCurrent() {
            return maxCurrent;
        }

        public double getMaxSpeed() {
            return maxSpeed;
        }

        public double getTotalRevolutions() {
            return totalRevolutions;
        }

        public double getCurrentPosition() {
            return currentPosition;
        }

        public double getCurrentSpeed() {
            return currentSpeed;
        }
    }

    private SystemMonitor() {
        motors = new HashMap<>();
        motorStats = new HashMap<>();
        motorMonitorTable = NetworkTableInstance.getDefault().getTable("SystemMonitor");
        canBusTable = motorMonitorTable.getSubTable("CAN Bus");
        pdp = new PowerDistribution(55,ModuleType.kRev);
    }

    public static SystemMonitor getInstance() {
        if (instance == null) {
            instance = new SystemMonitor();
        }
        return instance;
    }

    public void registerMotor(String name, CANSparkMax motor) {
        motors.put(name, motor);
        motorStats.put(name, new MotorStats());
    }

    public void monitorMotor(String name) {
        CANSparkMax motor = motors.get(name);
        MotorStats stats = motorStats.get(name);
        NetworkTable motorTable = motorMonitorTable.getSubTable(name);

        double current = motor.getOutputCurrent();
        double temp = motor.getMotorTemperature();
        double speed = motor.get();
        int faults = motor.getFaults();

        // Update stats
        stats.updateMaxCurrent(current);
        stats.updateMaxSpeed(speed);
        stats.updateTotalRevolutions(motor.getEncoder().getPosition());

        if (current > HIGH_CURRENT_THRESHOLD) {
            motorTable.getEntry("Warning").setString("High current");
        }

        if (temp > HIGH_TEMP_THRESHOLD) {
            motorTable.getEntry("Warning").setString("High temperature");
        }

        if (faults > 0) {
            motorTable.getEntry("Warning").setString("Motor fault");
        }

        // Post motor data and stats to NetworkTable
        motorTable.getEntry("Current").setDouble(current);
        motorTable.getEntry("Temperature").setDouble(temp);
        motorTable.getEntry("Faults").setDouble(faults);
        motorTable.getEntry("Max Current").setDouble(stats.getMaxCurrent());
        motorTable.getEntry("Max Speed").setDouble(stats.getMaxSpeed());
        motorTable.getEntry("Total Revolutions").setDouble(stats.getTotalRevolutions());
    }

    public void montorPowerDistribution() {
        double currentTime = RobotController.getFPGATime() / 1000000.0;
        if(lastPdpUpdateTime == 0) {
            lastPdpUpdateTime = currentTime;
        }
        double timeDelta = (currentTime - lastPdpUpdateTime); // in seconds
        lastPdpUpdateTime = currentTime;

        double voltage = pdp.getVoltage();

        if(pdpActiveChannels == null) {
            pdpActiveChannels = new boolean[pdp.getNumChannels()];
        }
        if(pdpRunningAverageAmps == null) {
            pdpRunningAverageAmps = new double[pdp.getNumChannels()];
        }
        if(pdpMaxAmps == null) {
            pdpMaxAmps = new double[pdp.getNumChannels()];
        }
        if(pdpTotalEnergy == null) {
            pdpTotalEnergy = new double[pdp.getNumChannels()];
        }

        // Check connection
        motorMonitorTable.getSubTable("PDP").getEntry("Temperature").setDouble(pdp.getTemperature());
        motorMonitorTable.getSubTable("PDP").getEntry("Voltage").setDouble(voltage);
        for (int i = 0; i < pdp.getNumChannels(); i++) {
            double amps = pdp.getCurrent(i);
            double watts = voltage * amps;
            // energy in watt hours
            double wattHours = watts * timeDelta / 3600.0;


            // Channel name needs to have a two digit number
            String channelAmpLabel = String.format("Amps %02d", i);
            String channelRunningAverageLabel = String.format("Amps Ave %02d", i);
            String channelWattHoursLabel = String.format("WattHours %02d", i);
            String channelMaxAmpsLabel = String.format("MaxAmps %02d", i);



            // Compute running average
            pdpRunningAverageAmps[i] = 0.99 * pdpRunningAverageAmps[i] + 0.01 * amps;

            // Compute total energy
            pdpTotalEnergy[i] += wattHours;

            // Compute max amps
            if(amps > pdpMaxAmps[i]) {
                pdpMaxAmps[i] = amps;
            }

            // Set the channel as active if the running average is over 0.1A
            // Once it is active it is always active
            if(pdpRunningAverageAmps[i] > 0.1) {
                pdpActiveChannels[i] = true;
            } 

            if(pdpActiveChannels[i]) {
                // compute the current amps, total energy, max amps, running average amps, teh total energy.
                //motorMonitorTable.getSubTable("PDP").getSubTable("Amps").getEntry(channelRunningAverageLabel).setDouble(amps);
                //motorMonitorTable.getSubTable("PDP").getSubTable("AmpsRunningAve").getEntry(channelRunningAverageLabel).setDouble(pdpRunningAverageAmps[i]);
                //motorMonitorTable.getSubTable("PDP").getSubTable("MaxAmps").getEntry(channelMaxAmpsLabel).setDouble(pdpMaxAmps[i]);
                motorMonitorTable.getSubTable("PDP").getSubTable("Energy").getEntry(channelWattHoursLabel).setDouble(wattHours);

            }
        }
    }

    public void monitorAllMotors() {
        for (String name : motors.keySet()) {
            monitorMotor(name);
        }
    }

    public void scanCANBus() {
        CANStatus canStatus = RobotController.getCANStatus();

        canBusTable.getEntry("Percent Bus Utilization").setDouble(canStatus.percentBusUtilization);
        canBusTable.getEntry("Bus Off Count").setDouble(canStatus.busOffCount);
        canBusTable.getEntry("TX Full Count").setDouble(canStatus.txFullCount);
        canBusTable.getEntry("Receive Error Count").setDouble(canStatus.receiveErrorCount);
        canBusTable.getEntry("Transmit Error Count").setDouble(canStatus.transmitErrorCount);
    }

    @Override
    public void periodic() {
        monitorAllMotors();
        scanCANBus();
        montorPowerDistribution();

        // Check robot rio CPU usage
        motorMonitorTable.getEntry("CPU Temperature").setDouble(RobotController.getCPUTemp());
        motorMonitorTable.getEntry("Memory Usage").setDouble(RobotController.getBatteryVoltage());
        motorMonitorTable.getEntry("Battery Voltage").setDouble(RobotController.getBatteryVoltage());


        

    }
}