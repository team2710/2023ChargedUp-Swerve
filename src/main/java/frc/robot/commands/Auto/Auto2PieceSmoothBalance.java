package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.AutoBalanceSwerve.RobotDirectionToStation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Auto2PieceSmoothBalance extends SequentialCommandGroup {
    
    public Auto2PieceSmoothBalance(SwerveAutoBuilder autoBuilder) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "2 Piece Smooth + Balance", new PathConstraints(4.8, 5));

        addCommands(
            autoBuilder.fullAuto(pathGroup)
        );
    }

}
