package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;
import edu.wpi.first.wpilibj.Joystick;

public class FlywheelJoystick extends CommandBase {
  /** Creates a new FlywheelJoystick. */

private final Flywheel roller;
private final Joystick joy;

  public FlywheelJoystick(Flywheel roller, Joystick joy) {
    this.roller = roller;
    this.joy = joy;
    addRequirements(flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      roller.setRollerPower(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roller.setRollerPower(-joy.getRawAxis(1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roller.setRollerPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
