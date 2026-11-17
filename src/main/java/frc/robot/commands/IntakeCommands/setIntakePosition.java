package frc.robot.commands.IntakeCommands;

import org.wpilib.command2.Command;
import frc.robot.subsystems.Intake.IntakePivot;


public class setIntakePosition extends Command {
    IntakePivot pivot;
    double pos;

  
    public setIntakePosition(double position) {
        pivot = IntakePivot.getInstance();
        this.pos = position;

        addRequirements(pivot);

    }

    @Override
    public void initialize() {
        pivot.setExtenderPosition(pos);
        pivot.postStatus("going to: " + pos);
    }

    @Override
    public void execute() {
        pivot.setTargetPosition();

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return pivot.atTargetPosition();
    }
}
