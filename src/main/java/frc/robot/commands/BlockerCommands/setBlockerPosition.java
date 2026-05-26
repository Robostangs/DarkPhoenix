package frc.robot.commands.BlockerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Intake.IntakePivot;

public class setBlockerPosition extends Command {
    Blocker blocker;
    double pos;
    boolean isBack;

    public setBlockerPosition(double position, boolean isBack) {
        blocker = Blocker.getInstance();
        this.pos = position;
        this.isBack = isBack;

        addRequirements(blocker);

    }

    @Override
    public void initialize() {
        if (isBack) {
            blocker.setDC(Constants.BlockerConstants.kDutyCycle);

        } else {
            blocker.setDC(-Constants.BlockerConstants.kDutyCycle);

        }
        blocker.postStatus("going to: " + pos);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        blocker.setDC(0);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(blocker.getBlockerPos() - pos) < 0.05;
    }
}
