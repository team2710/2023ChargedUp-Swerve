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

public class AutoCubeTaxi extends SequentialCommandGroup {
    
    public static enum Side {
        SMOOTH, BUMP
    }

    public AutoCubeTaxi(SwerveAutoBuilder autoBuilder, Side side) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "Cube + Taxi " + (side == Side.SMOOTH ? "Smooth" : "Bump"), new PathConstraints(2, 3));

        addCommands(
            autoBuilder.fullAuto(pathGroup)
        );
    }

}
