package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveDrive;

public class LeaveZone extends Command {
    private final SwerveDrive driveSubsystem;
    private final Timer timer = new Timer();
    private static final double DRIVE_DURATION = 2.0; // 2 seconds

    public LeaveZone(SwerveDrive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(0.75, 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(DRIVE_DURATION);
    }
}