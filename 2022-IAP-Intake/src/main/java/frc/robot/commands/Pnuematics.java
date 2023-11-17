// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class  extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PnuematicsDS pneumaticsds;
  private final Compressor comp = new Compresser();
  private final DoubleSoleniod soleniod = new DoubleSoleniod(0,1);
  
  private final Joystick stick = new Joystick(0);

  private final XboxController xbox = new XboxController(1);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Pnuematics (PneumaticsDS pnuematicsds, XboxController xbox) {
    this.xbox = xbox;
    this.pneumaticsds = pneumaticsds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pneumaticsds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    comp.stop();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if(xbox.getBumper(Hand.kLeft)){
      soleniod.set(DoubleSolendiod.Value.kForward);
    }else if(xbox.getBumper(Hand.kRight)){
      soleniod.set(DoubleSolendiod.Value.kReverse);
    }

    if(xbox.getAButton()){
      comp.start();

    }else if(xbox.getBButton()){
      comp.stop();
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
