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

public class AutoCubeTaxiBalance extends SequentialCommandGroup {
    
    public AutoCubeTaxiBalance(SwerveAutoBuilder autoBuilder) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "Cube + Taxi + Balance", new PathConstraints(2, 3));

        addCommands(autoBuilder.fullAuto(pathGroup));
    }

}
