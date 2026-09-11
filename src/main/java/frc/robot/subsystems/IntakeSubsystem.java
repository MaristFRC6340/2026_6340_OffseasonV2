// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake Subsystem. */
  TalonFX intakeRoller;
  TalonFX intakeRollerLeft;
  TalonFX intakePivot;

  double pivotPos = 0;

  PositionVoltage pos_request = new PositionVoltage(0).withSlot(0); //set motor's pos setpoint to pos specified 7

  public IntakeSubsystem() {
    intakeRoller = new TalonFX(IntakeConstants.intakeID);
    intakeRollerLeft = new TalonFX(IntakeConstants.intakeIDLeft);
    intakePivot = new TalonFX(IntakeConstants.pivotMotorID);
    // Set Config

    SmartDashboard.putNumber("Pivot Position", 0);

  }

  public void periodic(){
    pivotPos = intakePivot.getPosition().getValue().magnitude();
    SmartDashboard.putNumber("Pivot Position", pivotPos);
  }

  public void setRollerSpeed(double speed) {
    intakeRoller.set(speed);
    intakeRollerLeft.set(speed);
  }
  public void setPivotPos(double pos) {
    intakePivot.setControl(pos_request.withPosition(pos));
  }
  public void setPivotSpeed(double speed) {
    intakePivot.set(speed);
  }

   //Commands 
  // Intake Commands
  public Command stopRollerCommand(){
    return this.runOnce(() -> setRollerSpeed(0));
  }

  public Command setRollerSpeedCommand(double speed) {
    return Commands.startEnd(
      () -> setRollerSpeed(speed),
      () -> setRollerSpeed(0)
    );
  }

  // Pivot Commands
 public Command intakeDownCommand() {
    return Commands.runOnce(() -> this.setPivotPos(3)); //got -5 from old 6340 robot we might need to change
 }
 public Command intakeUpCommand() {
  return Commands.runOnce(() -> this.setPivotPos(0));
 }
 public Command setIntakePositionCommand(double pos) {
  return Commands.runOnce(() -> this.setPivotPos(pos));
 }
 public Command setIntakePivotSpeedCommand(double speed) {
   return Commands.startEnd(
    () -> setPivotSpeed(speed),
    () -> setPivotSpeed(0)
   );
 }
 public Command stopIntakePivotSpeed() {
    return this.runOnce(() -> setPivotSpeed(0));
 }
}
