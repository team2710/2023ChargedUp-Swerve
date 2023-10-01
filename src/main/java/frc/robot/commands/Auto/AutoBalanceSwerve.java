package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceSwerve extends SequentialCommandGroup {

    public static enum RobotDirectionToStation {
        AWAY, TOWARD
    }

    boolean isBalancing = false;

    public AutoBalanceSwerve(DriveSubsystem driveSubsystem, RobotDirectionToStation robotDirectionToStation) {
        double multiplier = (robotDirectionToStation == RobotDirectionToStation.AWAY ? -1 : 1);
        double initialSpeed = 0.5 * multiplier;
        double reverseSpeed = 0.25 * multiplier * -1;
        
        addCommands(
            new InstantCommand(() -> {
                driveSubsystem.forwardDrive(initialSpeed);
            }), new RunCommand(() -> {
                if (Math.abs(driveSubsystem.getGyro().getRoll()) > 1) {
                    driveSubsystem.forwardDrive(reverseSpeed * Math.signum(driveSubsystem.getGyro().getRoll()));
                    isBalancing = true;
                } else if (isBalancing) {
                    driveSubsystem.forwardDrive(reverseSpeed);
                }
            }, driveSubsystem)
        );
    }

}
