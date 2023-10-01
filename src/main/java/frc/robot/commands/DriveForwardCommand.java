package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCommand extends CommandBase {
    
    double speed;
    DriveSubsystem driveSubsystem;

    public DriveForwardCommand(DriveSubsystem driveSubsystem, double speed) {
        this.speed = speed;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void execute() {
        driveSubsystem.forwardDrive(speed);
    }

}
