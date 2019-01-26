/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;




public class ArcadeDrive extends Command {
  private double LeftPower, RightPower;


  public ArcadeDrive() {
    requires(Robot.drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    LeftPower = Robot.m_oi.getLeftX() + Robot.m_oi.getRightY();
    RightPower = -Robot.m_oi.getLeftX() + Robot.m_oi.getRightY();
    
    Robot.drive.setPower(LeftPower * 0.5, RightPower*0.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
