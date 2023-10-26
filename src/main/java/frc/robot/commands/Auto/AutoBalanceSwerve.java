package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        double initialSpeed = 1.5 * multiplier;
        double reverseSpeed = 0.7 * multiplier;
        
        addCommands(
            new InstantCommand(() -> {
                driveSubsystem.forwardDrive(initialSpeed);
            }), new RunCommand(() -> {
                SmartDashboard.putNumber("Gyro", driveSubsystem.getGyro().getRoll());
                if (driveSubsystem.getGyro().getRoll() > 5) {
                    driveSubsystem.forwardDrive(1.5 * multiplier);
                    isBalancing = true;
                } else if (driveSubsystem.getGyro().getRoll() < -7) {
                    driveSubsystem.forwardDrive(-0.5 * multiplier);
                } else if (isBalancing) {
                    driveSubsystem.forwardDrive(0);
                }
            }, driveSubsystem)
        );
    }

}
