// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveSubsystem extends SubsystemBase {
  // 4 Swerve Modules
  private final MAXSwerveModule[] modules = new MAXSwerveModule[4];   // FL, FR, BL, BR

  // 1 Gyro
  private final AHRS gyro;

  // poseEstimator to estimate the robot's position on the field
  private Rotation2d rawGyroRotation = new Rotation2d();

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, 
                                   rawGyroRotation, 
                                   lastModulePositions, 
                                   new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Instantiate 4 modules
    // FL
    modules[0] = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId, 
      DriveConstants.kFrontLeftTurningCanId, 
      DriveConstants.kFrontLeftChassisAngularOffset);

    // FR
    modules[1] = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId, 
      DriveConstants.kFrontRightTurningCanId, 
      DriveConstants.kFrontRightChassisAngularOffset);

    // BL
    modules[2] = new MAXSwerveModule(
      DriveConstants.kBackLeftDrivingCanId, 
      DriveConstants.kBackLeftTurningCanId, 
      DriveConstants.kBackLeftChassisAngularOffset);

    // BR
    modules[3] = new MAXSwerveModule(
      DriveConstants.kBackRightDrivingCanId, 
      DriveConstants.kBackRightTurningCanId, 
      DriveConstants.kBackRightChassisAngularOffset);

    // Instantiate gyro
    gyro = new AHRS(NavXComType.kMXP_SPI);

    // zero out the gyro's heading
    // Create a new thread so gyro calibration won't delay other robot's operations
    new Thread( () -> {
      try {
        Thread.sleep(1000);     // wait 1 sec (1000 millisec) for gyro to calibrate before zero out the heading
        zeroHeading();
      } catch (Exception e) {
        DriverStation.reportError("Error calibrating Navx MXP: " + e.getMessage(), true);
      }
    }).start();

    // Configure AutoBuilder for PathPlanner
    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          Constants.DriveConstants.translationConstants,
          Constants.DriveConstants.rotationConstants
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading )deg)", getHeading());
    // Display robot's location
    SmartDashboard.putString("Robot Location" , getPose().getTranslation().toString());

    // Read wheel positions
    SwerveModulePosition[] modulePositions = getModulePositions();

    // and deltas from each module
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] = new SwerveModulePosition(modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                                                           modulePositions[moduleIndex].angle);
      // save the last position
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    rawGyroRotation = getRotation2d();

    // Apply odometry update
    poseEstimator.update(rawGyroRotation, modulePositions);

  }

  // FOR GYRO =======================================================================================================
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /*
   * Command factory method zeroHeadingCommand
   * @return a command
   */
  public Command zeroHeadingCommand(){
    // Inline construction of command goes here
    // Subsystem: RunOnce implicitly requires 'this' subsystem
    return runOnce(
      () -> {
        zeroHeading();
      }
    );
  }

  /**
   * Returns the heading of the robot in degrees
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getYaw();
  }

  /**
   * Returns the heading of the robot in Rotation2d format
   *
   * @return the robot's heading in Rotation2d, angle from -180 to 180
   */
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  // FOR DRIVESUBSYSTEM ================================================================================================
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Renormalizes the wheel speeds if any individual speed is above the specified maximum
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Apply new desired states to each wheel
    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    modules[0].resetEncoders();
    modules[1].resetEncoders();
    modules[2].resetEncoders();
    modules[3].resetEncoders();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationUnitsPerSecond);
    SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationUnitsPerSecond);
    SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAccelerationUnitsPerSecond);

    // 1. Apply Deadband
    xSpeed = MathUtil.applyDeadband(xSpeed, OperatorConstants.kDriveDeadband);
    ySpeed = MathUtil.applyDeadband(ySpeed, OperatorConstants.kDriveDeadband);
       rot = MathUtil.applyDeadband(rot, OperatorConstants.kDriveDeadband);

    // 2. Make sure speeds don't exceed the maximum theoritical values
    xSpeed = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
       rot = rot * DriveConstants.kMaxAngularSpeed;

    // 3. Make driving smoother
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    rot = rotLimiter.calculate(rot);

    // 4. Construct desired chassis speeds
    // chassisSpeeds relative to robot
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    // chassisSpeeds relative to the field
    if (fieldRelative) chassisSpeeds.toRobotRelativeSpeeds(getRotation2d());

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    setModuleStates(swerveModuleStates);

  }

  // FOR PoseEstimator ===============================================================================================
  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    System.out.println(pose);
    poseEstimator.resetPose(pose);
  }

  public ChassisSpeeds getSpeeds() {
    // get the resulting chassis speed.
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    robotRelativeSpeeds.discretize(0.02);
    ChassisSpeeds targetSpeeds = robotRelativeSpeeds;

    // An array containing the module states.
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  // setStates is replaced by setModuleStates()
  /*
  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }
  */

  /* Not used, since setStates() is replaced by setModuleStates()
  public void setTargetState(SwerveModuleState targetState) {
    // Optimize the state
    currentState = SwerveModuleState.optimize(targetState, currentState.angle);

    currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
  }
  */

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

}
