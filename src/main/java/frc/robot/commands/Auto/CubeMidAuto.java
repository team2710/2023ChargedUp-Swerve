package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class CubeMidAuto extends SequentialCommandGroup {
    public CubeMidAuto(Arm arm, Elevator elevator, Intake intake) {
        addCommands(
            new ArmMoveCommand(arm, ArmConstants.kCubeMid),
            new ElevatorMoveCommand(elevator, ElevatorConstants.kCubeMid),
            new WaitCommand(2),
            new IntakeCommand(intake, 25, ArmConstants.kIntake),
            new WaitCommand(1),
            new IntakeCommand(intake, 25, 0),
            new WaitCommand(1),
            new ElevatorMoveCommand(elevator, 0),
            new ArmMoveCommand(arm, 0)
        );
    }
}

