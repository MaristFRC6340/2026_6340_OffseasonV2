// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake Subsystem. */
  TalonFX intakeRoller;
  TalonFX intakeRollerLeft;
  TalonFX intakePivot;

  double pivotPos = 0;

  PositionVoltage pos_request = new PositionVoltage(0).withSlot(0); //set motor's pos setpoint to pos specified 7

  // Magic Motion Request
  private MotionMagicVoltage m_pivot_request = new MotionMagicVoltage(0);

  public IntakeSubsystem() {
    intakeRoller = new TalonFX(IntakeConstants.intakeID);
    intakeRollerLeft = new TalonFX(IntakeConstants.intakeIDLeft);
    intakePivot = new TalonFX(IntakeConstants.pivotMotorID);
    // Set Config

    //var pivotConfigs = new TalonFXConfiguration();
    //pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //intakePivot.getConfigurator().apply(pivotConfigs);

    // Magic Motion Configuration for Pivot Motor
    var talonFXConfigs = new TalonFXConfiguration();
    // Slot 0 Gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kA = 0;
    slot0Configs.kG = 0.3;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kP = 3;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    // Motion Magic Settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 8; // 10 rpm of Drive Motor
    motionMagicConfigs.MotionMagicAcceleration = 50; // 50 rps acceleration
    motionMagicConfigs.MotionMagicJerk = 20; // Target Jerk of 20 rps
    
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply to pivot motor
    intakePivot.getConfigurator().apply(talonFXConfigs);



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
    intakePivot.setControl(m_pivot_request.withPosition(pos));
    //intakePivot.setControl(pos_request.withPosition(pos));
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
    return Commands.runOnce(() -> this.setPivotPos(12)); // Deployed Postion
 }
 public Command intakeUpCommand() { 
  return Commands.runOnce(() -> this.setPivotPos(0)); // Up Position inside Robot
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
