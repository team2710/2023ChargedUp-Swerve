package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class TestPath extends SequentialCommandGroup {
    
    public TestPath(SwerveAutoBuilder autoBuilder, DriveSubsystem swerve, Arm arm, Elevator elevator, Intake intake) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "3 Piece Smooth", new PathConstraints(1, 3));
        
        addCommands(
            autoBuilder.followPathGroup(pathGroup)
        );
    }

}
