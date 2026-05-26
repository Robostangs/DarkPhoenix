package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePivot;


public class MovePivot extends Command {
    private final IntakePivot pivot;
    private final DoubleSupplier dutyCycle;

    
    public MovePivot(DoubleSupplier dutyCycle) {
        pivot = IntakePivot.getInstance();
        this.dutyCycle = dutyCycle;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pivot.setDC(dutyCycle.getAsDouble());
        pivot.postStatus("Moving pivot at duty cycle: " + dutyCycle.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // This command runs until explicitly cancelled.
        return false;
    }
}
