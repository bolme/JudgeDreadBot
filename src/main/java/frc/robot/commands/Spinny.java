package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class Spinny extends Command {

    SwerveDrive drive = SwerveDrive.getInstance();
    int count = 0;

    public Spinny() {

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
        drive.drive(.5, -.5, -2.5 , true);
        count = count + 1;
    }

}
