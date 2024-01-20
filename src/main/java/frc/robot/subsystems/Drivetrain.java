// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Victor SPX list
  private WPI_VictorSPX rightLead;
  private WPI_VictorSPX rightFollow;
  private WPI_VictorSPX leftLead;
  private WPI_VictorSPX leftFollow;

  //Encoders
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  //Sims
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;

  private int direction = 1;
  
  // path following
  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;

  //Gyro
  public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  //Sims
  private ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);
  
  DifferentialDrive diffDrive;
  DifferentialDrivetrainSim drivetrainSim;

  //Create a chooser for selecting speed
  SendableChooser<Double> driveScaleChooser = new SendableChooser<Double>();
  public double CURRENT_DRIVE_SCALE;

  //dash
  public final Field2d field2d;

  public Drivetrain() {

    //create motor objects
    rightLead = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_LEAD_ID);
    rightFollow = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_FOLLOW_ID);
    leftLead = new WPI_VictorSPX(Constants.DriveConstants.LEFT_LEAD_ID);
    leftFollow = new WPI_VictorSPX(Constants.DriveConstants.LEFT_FOLLOW_ID);

    //ensure motor controllers are in default config before configuration
    rightLead.configFactoryDefault();
    rightFollow.configFactoryDefault();
    leftLead.configFactoryDefault();
    leftFollow.configFactoryDefault();

    //neautral mode of motors brake/coast
    rightLead.setNeutralMode(NeutralMode.Brake);
    rightFollow.setNeutralMode(NeutralMode.Brake);
    leftLead.setNeutralMode(NeutralMode.Brake);
    leftFollow.setNeutralMode(NeutralMode.Brake);

    //motor inversion
    rightLead.setInverted(DriveConstants.RIGHT_DRIVE_INVERT);
    rightFollow.setInverted(DriveConstants.RIGHT_DRIVE_INVERT);
    leftLead.setInverted(DriveConstants.LEFT_DRIVE_INVERT);
    leftFollow.setInverted(DriveConstants.LEFT_DRIVE_INVERT);

    //group the motors
    rightFollow.follow(rightLead);
    leftFollow.follow(leftLead);

    //Diferential Drive
    diffDrive = new DifferentialDrive(rightLead, leftLead);
    //Sim
    drivetrainSim = new DifferentialDrivetrainSim(
      DCMotor.getCIM(2),
      RobotConstants.GEAR_RATIO,
      RobotConstants.MOTION_OF_INERTIA,
      RobotConstants.MASS_OF_ROBOT,
      RobotConstants.WHEEL_DIAMETER/2,
      RobotConstants.TRACK_WIDTH,
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );

    //initialize the drivetrain encoders
    leftEncoder = new Encoder(Constants.SensorConstants.LEFT_ENCODER_ID, Constants.SensorConstants.LEFT_ENCODER_BUSID, Constants.DriveConstants.LEFT_DRIVE_INVERT);
    rightEncoder = new Encoder(Constants.SensorConstants.RIGHT_ENCODER_ID, Constants.SensorConstants.RIGHT_ENCODER_BUSID, Constants.DriveConstants.RIGHT_DRIVE_INVERT);
    leftEncoder.setDistancePerPulse(Constants.RobotConstants.WHEEL_CIRCUM/Constants.SensorConstants.ENCODER_COUNTS_PER_ROT);
    resetEncoders();
    //Sims
    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);

    //Odometry and Kinematics
    odometry = new DifferentialDriveOdometry(getRotation2D(), getLeftDistance(), getRightDistance());
    kinematics = new DifferentialDriveKinematics(Constants.RobotConstants.TRACK_WIDTH);

    //Drive scale option
    driveScaleChooser.addOption("100%", 1.0);
    driveScaleChooser.setDefaultOption("75%", 0.75);
    driveScaleChooser.addOption("50%", 0.5);
    driveScaleChooser.addOption("25%", 0.25);

    SmartDashboard.putData("Drive Speed", driveScaleChooser);

    //dash
    field2d = new Field2d();
    SmartDashboard.putData(field2d);
  }

  //tank drive
  public void tankDrive(double right, double left){
    diffDrive.tankDrive(direction * left, direction * right);
  }

  public void arcadeDrive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void stopMotors(){
    rightLead.set(0);
    rightFollow.set(0);
    leftLead.set(0);
    leftFollow.set(0);
  }

  public Rotation2d getRotation2D(){
    return gyro.getRotation2d();
  }

  //encoder distance in millimeters
  public double getLeftDistance() {
    return leftEncoder.getDistance() / 1000;
  }
  public double getRightDistance() {
    return rightEncoder.getDistance() / 1000;
  }
  //reset Encoders
  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  //Odometry and Kinematics
  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }
  public void updateOdometry(){
    odometry.update(getRotation2D(), getLeftDistance(), getRightDistance());
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void toggleDirection(){
    this.direction *= -1;
  }

  public void simulationPeriodic(){
    drivetrainSim.setInputs(direction, CURRENT_DRIVE_SCALE);
  }

  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumber("Left Drive Encoder", leftEncoder.getRaw());
    SmartDashboard.putNumber("Right Drive Encoder", rightEncoder.getRaw());

    SmartDashboard.putNumber("XPos", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("YPos", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Heading", getRotation2D().getDegrees());

    SmartDashboard.putNumber("Angle", gyro.getAngle());

    field2d.setRobotPose(odometry.getPoseMeters());

    CURRENT_DRIVE_SCALE = driveScaleChooser.getSelected();
  }
}
