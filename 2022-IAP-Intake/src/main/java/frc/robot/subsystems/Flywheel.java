// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel system. */

  private double flywheelTolerance = 0.05; // Tolerance of PID controller
  private boolean override = false; // Helps us switch from manual to auto
  private Timer overrideTimer = new Timer(); // We want a toggle button of some sorts
  private double overrideTime = 1.0;

  private ShuffleboardTab flyTab = Shuffleboard.getTab("Roller");
  GenericEntry nativeVel = flyTab.add("TickVelocity", 0.0).getEntry();
  GenericEntry nativePos = flyTab.add("TickPos", 0.0).getEntry();
  GenericEntry rpmEntry = flyTab.add("RPM", 0.0).getEntry();


  private final WPI_TalonSRX roller = new WPI_TalonSRX(Constants.ShooterPorts.rollerPort);



  public Flywheel() {

     // Assigns simulation and motor objects

    roller.configFactoryDefault();
    //Start and reset the timer
    overrideTimer.start(); // Start timer
    overrideTimer.reset(); // Reset timer

  } 
  

  public void setRollerPower(double speed) {
    roller.set(speed);
  }
  

  public double getRollerRPM() {
    return ((roller.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  
  public double getRollerPower() {
    return roller.get();
  }
     
  private int velocityToNativeUnits(double velocityRPM) {
    double motorRotationsPerSecond = velocityRPM / 60.0;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * 4096.0);
    return sensorCountsPer100ms;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Average RPM", getRollerRPM());
    SmartDashboard.putNumber("Left Flywheel RPM", getRollerRPM());
    SmartDashboard.putNumber("Left Flywheel Power", getRollerPower());

    if (RobotContainer.getJoy1().getRawButton(2) && overrideTimer.get() >= overrideTime) {
      override = !override;
      overrideTimer.reset();
      }
  }

}