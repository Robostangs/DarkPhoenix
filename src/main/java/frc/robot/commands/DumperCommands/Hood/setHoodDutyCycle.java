package frc.robot.commands.DumperCommands.Hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dumper.Hood;

public class setHoodDutyCycle extends Command {

  /*
   * This reads the input from the manip controller and sets the hood to that
   * The hood is first said to be in state of manual adjustment
   * Then gets the input from the joystick constantly
   * When the code ends, the hood moves at {@code hoodDutyCycle} speed and its
   * status is stopped
   */

  Hood hood;
  DoubleSupplier speed;

  public setHoodDutyCycle(DoubleSupplier speed) {
    hood = Hood.getInstance();
    this.speed = speed;
    addRequirements(hood);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    hood.setHoodDutyCycle(speed.getAsDouble());
    hood.postStatus("Setting hood duty cycle to " + speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
