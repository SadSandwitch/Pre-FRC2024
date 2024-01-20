// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command {
  private final Drivetrain drivetrain;
  private final XboxController xbox;
  SendableChooser<Boolean> drivChooser = new SendableChooser<Boolean>();

  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain drivetrain, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.xbox = xbox;
    addRequirements(drivetrain);

    drivChooser.setDefaultOption("Tank Drive", true);
    drivChooser.addOption("Arcade Drive", false);
    
    SmartDashboard.putData("Drive Mode", drivChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivChooser.getSelected()){ //tank
      double moveSpeedY = -(xbox.getLeftY()*drivetrain.CURRENT_DRIVE_SCALE);
      double moveSpeedX = -(xbox.getRightY()*drivetrain.CURRENT_DRIVE_SCALE);
      drivetrain.tankDrive(moveSpeedY, moveSpeedX);
    }else if(!(drivChooser.getSelected())){
      double speed = -(xbox.getLeftY()*drivetrain.CURRENT_DRIVE_SCALE);
      double turn = -(xbox.getLeftX()*drivetrain.CURRENT_DRIVE_SCALE);
      drivetrain.arcadeDrive(speed, turn);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
