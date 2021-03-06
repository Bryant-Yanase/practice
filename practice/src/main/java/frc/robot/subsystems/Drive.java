/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Teleop.ArcadeDrive;
import frc.robot.commands.Teleop.Tank_Drive;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {

  public static TalonSRX m_leftMaster;
  public static TalonSRX m_leftSlave1;
  public static TalonSRX m_leftSlave2;

  public static TalonSRX m_rightMaster;
  public static TalonSRX m_rightSlave1;
  public static TalonSRX m_rightSlave2;


  public Drive() {
    m_leftMaster = new TalonSRX(RobotMap.m_leftMaster);
    m_leftSlave1 = new TalonSRX(RobotMap.m_leftSlave1);
    m_leftSlave2 = new TalonSRX(RobotMap.m_leftSlave2);
    m_rightMaster = new TalonSRX(RobotMap.m_rightMaster);
    m_rightSlave1 = new TalonSRX(RobotMap.m_rightSlave1);
    m_rightSlave2 = new TalonSRX(RobotMap.m_rightSlave2);

    m_leftSlave1.follow(m_leftMaster);
    m_leftSlave2.follow(m_leftMaster);

    m_rightSlave1.follow(m_rightMaster);
    m_rightSlave2.follow(m_rightMaster);

    m_leftMaster.setInverted(true);
    m_leftSlave1.setInverted(true);
    m_leftSlave2.setInverted(true);
    m_rightMaster.setInverted(false);
    m_rightSlave1.setInverted(false);
    m_rightSlave2.setInverted(false);

    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_leftMaster.setSensorPhase(true);
    m_rightMaster.setSensorPhase(false);
  }

  public double getRightPosition () {
    return m_rightMaster.getSelectedSensorPosition();
    
  }

  public double getLeftPosition (){
    return m_leftMaster.getSelectedSensorPosition();

  }

  public double getAveragePosition(){
    return (getLeftPosition() + getRightPosition() / 2);

  }

  public void setPower(double RightPower, double LeftPower){
    m_leftMaster.set(ControlMode.PercentOutput, LeftPower);

    m_rightMaster.set(ControlMode.PercentOutput, RightPower);
  

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ArcadeDrive());
    setDefaultCommand(new Tank_Drive());
  }
}
