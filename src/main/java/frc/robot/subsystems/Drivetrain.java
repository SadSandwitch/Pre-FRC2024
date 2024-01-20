// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Victor SPX list
  WPI_VictorSPX rightLead;
  WPI_VictorSPX rightFollow;
  WPI_VictorSPX leftLead;
  WPI_VictorSPX leftFollow;

  DifferentialDrive diffDrive;

  //Shuffleboard
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

  public Drivetrain() {
    //create motor objects
    rightLead = new WPI_VictorSPX(0);
    rightFollow = new WPI_VictorSPX(0);
    leftLead = new WPI_VictorSPX(0);
    leftFollow = new WPI_VictorSPX(0);

    rightLead.configFactoryDefault();
    rightFollow.configFactoryDefault();
    leftLead.configFactoryDefault();
    leftFollow.configFactoryDefault();

    rightLead.setNeutralMode(NeutralMode.Brake);
    rightFollow.setNeutralMode(NeutralMode.Brake);
    leftLead.setNeutralMode(NeutralMode.Brake);
    leftFollow.setNeutralMode(NeutralMode.Brake);

    rightLead.setInverted(false);
    rightFollow.setInverted(false);
    leftLead.setInverted(true);
    leftFollow.setInverted(true);

    rightFollow.follow(rightLead);
    leftFollow.follow(leftLead);

    diffDrive = new DifferentialDrive(rightLead, leftLead);
  }

  public void tankDrive(double right, double left){
    diffDrive.tankDrive(left, right);
  }

  public void stopMotors(){
    rightLead.set(0);
    rightFollow.set(0);
    leftLead.set(0);
    leftFollow.set(0);
  }


  @Override
  public void periodic() {

  }
}
