package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.sensors.NavX;
import frc.robot.subsystems.SwerveDrive;

public class TeleopSwerveControl extends Command {

    private final SwerveDrive swerveDrive;
    private final XboxController controller;
    private boolean fieldOriented = true;
    private boolean squaredInputs = true;

    NetworkTable swerveControTable;

    private static final double DEADZONE = 0.1;

    private static TeleopSwerveControl instance = null;

    public static synchronized TeleopSwerveControl getInstance() {
        if (instance == null) {
            instance = new TeleopSwerveControl();
        }
        return instance;
    }

    public TeleopSwerveControl() {
        this.controller = RobotContainer.m_DriverController;
        swerveDrive = SwerveDrive.getInstance();

        swerveControTable = NetworkTableInstance.getDefault().getTable("TeleopSwerveControl");
       
        addRequirements(swerveDrive);
    }

    public void toggleFieldOriented() {
        fieldOriented = !fieldOriented;
    }

    public boolean getFieldOriented() {
        return fieldOriented;
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        if(controller.getStartButton()){
            NavX.getInstance().reset();
        }

        // TODO: Quick fix swaping the control  y should be forward and x should be strafe
        double x = -controller.getLeftY();
        double y = -controller.getLeftX();

        swerveControTable.getEntry("forward_input").setDouble(x);
        swerveControTable.getEntry("strafe_input").setDouble(y);

        // Compute the inputs as the direction and magnitude of the vector
        double magnitude = Math.sqrt(y * y + x * x);

        if(magnitude < DEADZONE){
            magnitude = 0;
            x = 0;
            y = 0;
        }
        else{
            x = x/magnitude;
            y = y/magnitude;
        }

        // Ensure the magnitude is within the range 0 to 1
        if (magnitude > 1) {
            magnitude = 1;
        }

        // Compute the rotation input
        double rotation = -controller.getRightX();
        swerveControTable.getEntry("rotation_input").setDouble(rotation);

        if(rotation <  DEADZONE && rotation > -DEADZONE){
            rotation = 0;
        }

        // Square the inputs (while preserving the sign) to increase fine control while permitting full power
        if(squaredInputs) {
            magnitude = magnitude * magnitude;
            rotation = Math.copySign(rotation*rotation, rotation);
        }

        // Compute the inputs as the direction and magnitude of the vector
        double forward = magnitude * x;
        double strafe = magnitude * y;

        // Compute the trigger input for precision controll
        double speedReductionTrigger = controller.getLeftTriggerAxis(); // range 0 to 1
        double turn_reduction = 1.0 - 0.5 * speedReductionTrigger;
        double drive_reduction = 1.0 - 0.5 * speedReductionTrigger;

        // Scale the inputs to the maximum speed
        forward = Constants.maxSpeed * drive_reduction * forward;
        strafe = Constants.maxSpeed * drive_reduction * strafe;
        rotation = Constants.maxChassisTurnSpeed * turn_reduction * rotation;

        swerveControTable.getEntry("forward_output").setDouble(forward);
        swerveControTable.getEntry("strafe_output").setDouble(strafe);
        swerveControTable.getEntry("rotation_output").setDouble(rotation);
        
        swerveControTable.getEntry("fieldOriented").setBoolean(fieldOriented);
        // Drive the robot
        swerveDrive.drive(forward, strafe, rotation, fieldOriented);
    }

    /**
     * Called when the command ends or is interrupted. Stops the swerve drive.
     *
     * @param interrupted true if the command was interrupted, false otherwise
     */
    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(0, 0, 0, fieldOriented);
    }

    /**
     * Determines whether the command has finished executing.
     * 
     * @return true if the command has finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}