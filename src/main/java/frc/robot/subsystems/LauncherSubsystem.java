// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.simulation.ironmaple.utils.LegacyFieldMirroringUtils2024;

public class LauncherSubsystem extends SubsystemBase {

    TalonFX leftShooter;
    TalonFX rightShooter;
    TalonFX hoodMotor; // Added by michaudc 04 Sep 26

    SparkFlex feederMotor;
    SparkFlex feederMotorRight;
    SparkFlex activeFloorFront;
    SparkFlex activeFloorBack;

    private double reverse = -1;
    private double forward =1;

    private final VelocityVoltage shooter_request = new VelocityVoltage(0).withSlot(0);
    PositionVoltage pos_request_Hood = new PositionVoltage(0).withSlot(0); //set motor's pos setpoint to pos specified 7


    //private final RelativeEncoder m_leftLaunchEncoder;
    //private final RelativeEncoder m_rightLaunchEncoder;
  /** Creates a new ExampleSubsystem. */
  public LauncherSubsystem() {
    leftShooter = new TalonFX(Constants.LauncherConstants.leftShooterID);
    leftShooter.setNeutralMode(NeutralModeValue.Coast);

    rightShooter = new TalonFX(Constants.LauncherConstants.rightShooterID);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);

    // Added michaudc 04 Sep 26
    hoodMotor = new TalonFX(Constants.LauncherConstants.hoodMotorID);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    // Shooter Motor Setup
    MotorOutputConfigs rightShooterConfigs = new MotorOutputConfigs();
    rightShooterConfigs.Inverted=InvertedValue.CounterClockwise_Positive;
    rightShooter.getConfigurator().apply(rightShooterConfigs);

    MotorOutputConfigs leftShooterConfigs = new MotorOutputConfigs();
    leftShooterConfigs.Inverted=InvertedValue.Clockwise_Positive;
    leftShooter.getConfigurator().apply(leftShooterConfigs);

    leftShooter.setNeutralMode(NeutralModeValue.Coast); // Set to Coast Mode: michaudc
    rightShooter.setNeutralMode(NeutralModeValue.Coast);

    rightShooter.getConfigurator().apply(Constants.LauncherConstants.launcherConfig); // Set to Configs in Contants
    leftShooter.getConfigurator().apply(Constants.LauncherConstants.launcherConfig);

    // Feeder Motors
    feederMotor = new SparkFlex(Constants.LauncherConstants.leftIndexerID, MotorType.kBrushless);
    feederMotorRight = new SparkFlex(Constants.LauncherConstants.rightIndexerID, MotorType.kBrushless);
    activeFloorFront = new SparkFlex(Constants.LauncherConstants.activeFloorFrontID, MotorType.kBrushless);// change from null to something else later
    activeFloorBack = new SparkFlex(Constants.LauncherConstants.activeFloorBackID, MotorType.kBrushless);// change from null to something else later

    


    // SparkMaxConfig launcherConfig = new SparkMaxConfig();
    // launcherConfig.smartCurrentLimit(Constants.LauncherConstants.launcherCurrentLimit);

        // launcherConfig.closedLoop
        // .p(0.00015)
        // .i(0)
        // .d(0)
        // .outputRange(0, 0.95)
        // .feedForward.kV( 12.0 / 6271 ); // 12 Volts divided by Maximum RPM of KrakenX60 (12.0 / 6271)
    
        //Configfor for feeder and active floor
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(Constants.LauncherConstants.launcherCurrentLimit);
        feederConfig.idleMode(IdleMode.kBrake);


        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederMotorRight.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        activeFloorFront.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        activeFloorBack.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        //launcherConfig.disableFollowerMode();

      // Invert Left
      //launcherConfig.follow(launcherRight); // Trying to have left follow the right
      //leftShooter.getConfigurator().apply(Constants.LauncherConstants.launcherConfig);

      // Encoders for Launching Motors
       //m_leftLaunchEncoder = leftShooter.getEncoder();
       //m_rightLaunchEncoder =  rightShooter.getEncoder();

    //   m_leftLaunchEncoder.setPosition(0);
    //   m_rightLaunchEncoder.setPosition(0);

      // Smart Dashboard
      SmartDashboard.putNumber("Left Launcher RPM", 0);
      SmartDashboard.putNumber("Right Launcher RPM:", 0);
      SmartDashboard.putNumber("Left Launch Amps", 0);
      SmartDashboard.putNumber("Right Launch Amps", 0);
      double rightShooterVelocity = rightShooter.getVelocity().getValueAsDouble()*60;
      double leftShooterVelocity = leftShooter.getVelocity().getValueAsDouble()*60;

      SmartDashboard.putNumber("Shoot Velocity Right", rightShooterVelocity);
      SmartDashboard.putNumber("Shoot Velocity Left", leftShooterVelocity);
      // binding camera

  }



  public void setRightShooterVelocity(double velocity){
    rightShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
  }

    public void setLeftShooterVelocity(double velocity){
    leftShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
  }

  public void setShooterVelocity(double velocity){
    //System.out.println("Shooting");
    rightShooter.setControl(shooter_request.withVelocity(-velocity).withFeedForward(0.5));
    leftShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
  }

  public void setFeederSpeed(double power) {
    feederMotor.set(-power);
    feederMotorRight.set(power);
  }



  public void setActiveFloorPower(double power){
    activeFloorBack.set(power);
    activeFloorFront.set(power);  }




    public void setIndexerAndFloorSpeed(double power) {
    feederMotorRight.set(power*2);
    feederMotor.set(reverse*(power*2));
    activeFloorBack.set(power*-1);
    activeFloorFront.set(power*-1);
  }

  //shooter below
    public void setShooterSpeed(double power){
      rightShooter.set(power);
      leftShooter.set(power);
  }

    public void setHoodPos(double pos) {
    // hoodMotor.setControl(pos_request_Hood.withPosition(pos));
        hoodMotor.set(pos);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updated by michaudc - resuse the key that already exists
    double rightShooterVelocity = rightShooter.getVelocity().getValueAsDouble()*60;
    double leftShooterVelocity = leftShooter.getVelocity().getValueAsDouble()*60;

    SmartDashboard.putNumber("Shoot Velocity Right", rightShooterVelocity);
    SmartDashboard.putNumber("Shoot Velocity Left", leftShooterVelocity);

    //TODO: Put the Positional Value of the Hood Motor

  }

  public Command setShooterVelocityCommand(double speed){
    return this.run(()-> setShooterVelocity(speed));
  }

  public Command stopShooterCommand(){
    return this.run(()-> setShooterSpeed(0));
  }

  public Command startFloorCommand(){
    return Commands.run(()-> setActiveFloorPower(.2));
  }

  public Command startStopFloorCommand() {
    return Commands.startEnd(() -> setActiveFloorPower(-0.2), 
    () -> setActiveFloorPower(0));
  }

    public Command stopFloorCommand(){
    return Commands.run(()-> setActiveFloorPower(0));
  }

  public Command feederSpeedCommand(double speed) {
    return Commands.run(() -> setFeederSpeed(speed));
  }

    public Command stopFeederCommand()  {
    return Commands.run(() -> setFeederSpeed(0));
  }


    public Command stopActiveFloorCommand(){
      return Commands.run(()-> setActiveFloorPower(0));
    }

  public Command startFeederCommand(){
    return Commands.run(()-> setFeederSpeed(0.2));
  }
  
    public Command reverseFeederCommand(){
    return Commands.run(()-> setFeederSpeed(-0.2));
  }

  public Command stopIndexerAndFloorCommand()  {
    return Commands.run(() -> setIndexerAndFloorSpeed(0));
  }

    public Command startIndexerAndFloorCommand()  {
    return Commands.run(() -> setIndexerAndFloorSpeed(0.8));
  }
  
    public Command reverseIndexerAndFloorCommand(){
    return Commands.run(()-> setFeederSpeed(-0.8));
  }
    // shooter below
    public Command setShooterSpeedCmd(double speed) {
    return this.startEnd(() -> {
              setShooterSpeed(speed);
       }, () -> {
           setShooterSpeed(0);
       });
    }

    public Command startShooter(){
      return Commands.run(()->setShooterSpeedCmd(0.8));
    }

    
    public Command hoodDownCommand() {
        return Commands.runOnce(() -> this.setHoodPos(-.1));   
    }

    public Command hoodUpCommand() {
      return Commands.runOnce(() -> this.setHoodPos(.1));
    }  

    public Command hoodStopCommand() {
      return Commands.runOnce(() -> this.setHoodPos(0));
    }  

}
