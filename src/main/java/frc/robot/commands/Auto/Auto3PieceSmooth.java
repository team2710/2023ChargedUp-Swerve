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

public class Auto3PieceSmooth extends SequentialCommandGroup {
    
    public Auto3PieceSmooth(SwerveAutoBuilder autoBuilder, DriveSubsystem swerve, Arm arm, Elevator elevator, Intake intake) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "3 Piece Smooth", new PathConstraints(4.8, 5));
        
        // addCommands(
        //     Commands.sequence(
        //         autoBuilder.resetPose(pathGroup.get(0)),

        //         // Score mid and start the first path
        //         Commands.parallel(
        //             elevator.moveCommand(ElevatorConstants.kCubeMid),
        //             arm.moveCommand(ArmConstants.kCubeMid)
        //         ),
        //         Commands.waitSeconds(0.5),
        //         intake.intakeCommand(ArmConstants.kIntake),
        //         Commands.parallel(
        //             elevator.moveCommand(0),
        //             arm.moveCommand(0),
        //             autoBuilder.followPathWithEvents(pathGroup.get(0))
        //         ),

        //         // Ground intake cone and start second path
        //         Commands.parallel(
        //             arm.moveCommand(ArmConstants.kGroundIntake),
        //             Commands.sequence(
        //                 intake.intakeCommand(ArmConstants.kIntake),
        //                 Commands.waitSeconds(1),
        //                 intake.intakeCommand(ArmConstants.kIntakeHold)
        //             ),
        //             autoBuilder.followPathWithEvents(pathGroup.get(1))
        //         ),

        //         // Score first cone mid and start third path
        //         Commands.parallel(
        //             elevator.moveCommand(ElevatorConstants.kConeMid),
        //             arm.moveCommand(ArmConstants.kConeMid)
        //         ),
        //         Commands.waitSeconds(1),
        //         intake.intakeCommand(ArmConstants.kOuttake),
        //         Commands.parallel(
        //             elevator.moveCommand(0),
        //             arm.moveCommand(0),
        //             autoBuilder.followPathWithEvents(pathGroup.get(2))
        //         ),

        //         // Ground intake cone and start fourth path
        //         Commands.parallel(
        //             arm.moveCommand(ArmConstants.kGroundIntake),
        //             Commands.sequence(
        //                 intake.intakeCommand(ArmConstants.kIntake),
        //                 Commands.waitSeconds(1),
        //                 intake.intakeCommand(ArmConstants.kIntakeHold)
        //             ),
        //             autoBuilder.followPathWithEvents(pathGroup.get(3))
        //         ),

        //         // Score cone mid
        //         Commands.parallel(
        //             elevator.moveCommand(ElevatorConstants.kCubeMid),
        //             arm.moveCommand(ArmConstants.kCubeMid)
        //         ),
        //         Commands.waitSeconds(1),
        //         intake.intakeCommand(ArmConstants.kOuttake)
        //     )
        // );
    }

}
