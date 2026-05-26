package frc.robot.commands.DumperCommands.Hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dumper.Hood;

public class setHoodPosition extends Command {
    Hood hood;
    DoubleSupplier hoodpos;
    boolean smort;

    public setHoodPosition() {
        hood = Hood.getInstance();
        this.smort = true;
        addRequirements(hood);
    }

    public setHoodPosition(DoubleSupplier hoodpos) {
        this.hoodpos = hoodpos;
        hood = Hood.getInstance();
        this.smort = false;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.postStatus("Setting hood position");
    }

    @Override
    public void execute() {
        double currentTarget;

        if (smort) {

            currentTarget = hood.getHoodAngle();
        } else {
            currentTarget = hoodpos.getAsDouble();
        }

        hood.setMotionMagic(currentTarget);

        hood.postStatus("Setting hood position to: " + currentTarget);

    }

    @Override
    public void end(boolean interrupted) {
        hood.postStatus("Hood position set");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
