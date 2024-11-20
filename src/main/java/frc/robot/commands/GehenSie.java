package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class GehenSie extends Command {

    SwerveDrive drive = SwerveDrive.getInstance();
    int count = 0;

    public GehenSie() {

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public boolean isFinished() {
        if (count > 250) {
            return true;
        }

        return false;
    

    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
    }

    @Override
    public void execute() {
        drive.drive(.5, 0, 0, false);
        count = count + 1;
    }

}
