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

    SparkMax feederMotor;
    SparkFlex feederMotorRight;
    SparkFlex activeFloorFront;
    SparkFlex activeFloorBack;
    private double reverse = -1;
    private double forward =1;

    private final VelocityVoltage shooter_request = new VelocityVoltage(0).withSlot(0);

    //private final RelativeEncoder m_leftLaunchEncoder;
    //private final RelativeEncoder m_rightLaunchEncoder;
  /** Creates a new ExampleSubsystem. */
  public LauncherSubsystem() {
    leftShooter = new TalonFX(Constants.LauncherConstants.leftShooterID);
    leftShooter.setNeutralMode(NeutralModeValue.Coast);

    rightShooter = new TalonFX(Constants.LauncherConstants.rightShooterID);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);

    // Upping Amp Limit for Shooter Motors

    MotorOutputConfigs rightShooterConfigs = new MotorOutputConfigs();
    rightShooterConfigs.Inverted=InvertedValue.CounterClockwise_Positive;
    rightShooter.getConfigurator().apply(rightShooterConfigs);

    MotorOutputConfigs leftShooterConfigs = new MotorOutputConfigs();
    leftShooterConfigs.Inverted=InvertedValue.Clockwise_Positive;
    leftShooter.getConfigurator().apply(leftShooterConfigs);


    feederMotor = new SparkMax(Constants.LauncherConstants.leftIndexerID, MotorType.kBrushless);
    feederMotorRight = new SparkFlex(Constants.LauncherConstants.rightIndexerID, MotorType.kBrushless);
    activeFloorFront = new SparkFlex(Constants.LauncherConstants.activeFloorFrontID, MotorType.kBrushless);// change from null to something else later
    activeFloorBack = new SparkFlex(Constants.LauncherConstants.activeFloorBackID, MotorType.kBrushless);// change from null to something else later


    var slot0ConfigsFlywheel = new Slot0Configs();
        slot0ConfigsFlywheel.kS = 0.1;
        slot0ConfigsFlywheel.kV = 0.12;
        slot0ConfigsFlywheel.kP = 0.11;
        slot0ConfigsFlywheel.kI = 0;
        slot0ConfigsFlywheel.kD = 0;
      rightShooter.getConfigurator().apply(slot0ConfigsFlywheel);
      leftShooter.getConfigurator().apply(slot0ConfigsFlywheel);


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
    rightShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
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
    feederMotorRight.set(reverse*power);
    feederMotor.set(power);
    activeFloorBack.set(reverse*power);
    activeFloorFront.set(power);
  }

  //shooter below
    public void setShooterSpeed(double power){
      rightShooter.set(power);
      leftShooter.set(power);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double rightShooterVelocityReal = rightShooter.getVelocity().getValueAsDouble()*60;
    SmartDashboard.putNumber("Shooter Velocity Right Real", rightShooterVelocityReal);

    double leftShooterVelocityReal = leftShooter.getVelocity().getValueAsDouble()*60;
    SmartDashboard.putNumber("Shooter Velocity Left Real", leftShooterVelocityReal);
  }

  public Command setShooterVelocityCommand(double speed){
    return this.run(()-> setShooterVelocity(speed));
  }

  public Command stopShooterCommand(){
    return this.run(()-> setShooterSpeed(0));
  }

  public Command startFloorCommand(){
    return Commands.run(()-> setActiveFloorPower(1));
  }

  public Command startStopFloorCommand() {
    return Commands.startEnd(() -> setActiveFloorPower(-0.75), 
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
    return Commands.run(()-> setFeederSpeed(0.8));
  }
  
    public Command reverseFeederCommand(){
    return Commands.run(()-> setFeederSpeed(-0.8));
  }

  public Command stopIndexerAndFloorCommand()  {
    return Commands.run(() -> setIndexerAndFloorSpeed(0));
  }

    public Command startIndexerAndFloorCommand()  {
    return Commands.run(() -> setIndexerAndFloorSpeed(1));
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
  
}
