package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class BiegenSieNachLinksAb extends Command {

    SwerveDrive drive = SwerveDrive.getInstance();
    int count = 0;

    public BiegenSieNachLinksAb() {

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public boolean isFinished() {
        if (count > 175) {
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
        drive.drive(1, 0, .5 , false);
        count = count + 1;
    }

}
