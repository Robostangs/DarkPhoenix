package frc.robot.commands.DumperCommands.Flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dumper.Flywheel;

public class ChargeUp extends Command {
    Flywheel shooter;
    DoubleSupplier targetRPS;
    boolean smort;
    boolean Gentle;

    public ChargeUp(boolean Gentle) {
        shooter = Flywheel.getInstance();
        smort = true;
        this.Gentle = Gentle;
        addRequirements(shooter);

    }

    public ChargeUp(DoubleSupplier targetRPS, boolean Gentle) {
        shooter = Flywheel.getInstance();
        this.targetRPS = targetRPS;
        smort = false;
        this.Gentle = false;
        addRequirements(shooter);

    }

    @Override
    public void initialize() {
        if(Gentle){
            shooter.setnewCurrentLimit(50);
        }


    }

    @Override
    public void execute() {
        double currentTarget;

        if (smort) {
            currentTarget = shooter.getFlywheelTargetVelocity();
        } else {
            currentTarget = targetRPS.getAsDouble();
        }

        shooter.setVelocity(currentTarget);

        SmartDashboard.putNumber("Flywheel/Meters", Robot.getMetersFromTarget());

        shooter.postStatus("Charging to " + currentTarget + " rps");

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVelocity(0);
        shooter.postStatus("Charged");
        shooter.setnewCurrentLimit(120); 

    }

}