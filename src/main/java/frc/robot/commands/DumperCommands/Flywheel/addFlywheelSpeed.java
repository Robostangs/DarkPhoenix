package frc.robot.commands.DumperCommands.Flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dumper.Flywheel;

public class addFlywheelSpeed extends Command {
    Flywheel flywheel;
    DoubleSupplier currentSpeed;

  public addFlywheelSpeed(DoubleSupplier addspeed) {
    flywheel = Flywheel.getInstance();
    this.currentSpeed = () -> flywheel.getFlywheelTargetVelocity() + addspeed.getAsDouble();
    addRequirements(flywheel);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    flywheel.setFlywheelDC(currentSpeed.getAsDouble());
    flywheel.postStatus("Setting flywheel duty cycle to " + currentSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  } 
}
}
