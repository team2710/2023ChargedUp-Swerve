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
    boolean isCharging = false;
    RobotDirectionToStation directionToStation;

    public AutoBalanceSwerve(DriveSubsystem driveSubsystem, RobotDirectionToStation robotDirectionToStation) {
        double multiplier = (robotDirectionToStation == RobotDirectionToStation.AWAY ? -1 : 1);
        double initialSpeed = 1 * multiplier;
        double reverseSpeed = 0.7 * multiplier;
        
        addCommands(
            new InstantCommand(() -> {
                driveSubsystem.forwardDrive(initialSpeed);
            }), new RunCommand(() -> {
                // if ((multiplier == -1 ? (driveSubsystem.getGyro().getRoll() > 3) : driveSubsystem.getGyro().getRoll() < -3) && !isCharging) {
                //     driveSubsystem.forwardDrive(0.1 * multiplier);
                //     isBalancing = true;
                // } else if (isBalancing) {
                //     driveSubsystem.forwardDrive(0);
                // }
                if (driveSubsystem.getGyro().getRoll() < -8) {
                    isBalancing = true;
                    driveSubsystem.forwardDrive(0.5);
                } else if (isBalancing) {
                    driveSubsystem.forwardDrive(0);
                    driveSubsystem.setX();
                }
            }, driveSubsystem)
        );
    }

}
