package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShiftCommand extends CommandBase {
    Command current;
    Command a, b;
    RobotContainer container;

    public ShiftCommand(Command a, Command b, RobotContainer container) {
        this.a = a;
        this.b = b;
        current = a;
        this.container = container;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void execute() {
        if (container.getIsControlShifted()) {
            current = b;
        } else {
            current = a;
        }
        current.execute();
    }
}
