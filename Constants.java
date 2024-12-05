// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
  * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
  * constants. This class should not be used for any other purpose. All constants should be declared
  * globally (i.e. public static). Do not put anything functional in this class.
  *
  * <p>It is advised to statically import this class (or one of its inner classes) wherever the
  * constants are needed, to reduce verbosity.
  */
  public final class Constants {

    // OperatorConstants -----------------------------------------------------------------------------
    public static class OperatorConstants {
      // Driver Controller Port number
      public static final int kDriverControllerPort = 0;

      // Deadband of joystick
      public static final double kDriveDeadband = 0.05;

    }

    // DriveConstants ------------------------------------------------------------------------------
    public static class DriveConstants {
      public static final int kFrontLeftDrivingCanId = 11;
      public static final int kFrontLeftTurningCanId = 3;
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2.0;     // MUST BE CALIBRATED

      public static final int kFrontRightDrivingCanId = 15;
      public static final int kFrontRightTurningCanId = 14;
      public static final double kFrontRightChassisAngularOffset = 0.0;               // MUST BE CALIBRATED

      public static final int kBackLeftDrivingCanId = 13;
      public static final int kBackLeftTurningCanId = 16;
      public static final double kBackLeftChassisAngularOffset = Math.PI;             // MUST BE CALIBRATED

      public static final int kBackRightDrivingCanId = 10;
      public static final int kBackRightTurningCanId = 4;
      public static final double kBackRightChassisAngularOffset = Math.PI / 2.0;      // MUST BE CALIBRATED

      /*
       * The formula for calculating the theoretical maximum velocity is:
       * <Motor free speed RPM> / 60 / <Drive Reduction> * <Wheel Diameter Meters> * pi
       * // 5676 / 60 / 6.75 * 0.102 * 3.14 = 4.5 m/sec
       */
      public static final double kMaxSpeedMetersPerSecond = NeoMotorConstants.kFreeSpeedRpm / 60.0
                                                            / ModuleConstants.kDrivingMotorReduction
                                                            * ModuleConstants.kWheelDiameterMeters
                                                            * Math.PI;
      
      // The theoretical maximum angular velocity
      public static final double kMaxAngularSpeed = 2.0 * Math.PI; 

      public static final double kMaxAccelerationUnitsPerSecond = 3.0;              // Units per second
      public static final double kMaxAngularAccelerationUnitsPerSecond = 3.0;       // Units per second
     
      // Chassis Configuration
      // Track Width: Distance between Center of right and left wheel on roobot
      public static final double kTrackWidth = Units.inchesToMeters(26.5);
      
      // Wheel Base: Distance between front and back wheel
      public static final double kWheelBase = Units.inchesToMeters(26.5);

      // SwerveDriveKinematics
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
      );
      
      public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
      public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);

    }

    // ModuleConstants -------------------------------------------------------------------------------
    public static class ModuleConstants {
      // Max. Current Limit for driving motor
      public static final int kDrivingMotorCurrentLimit = 50;    // Amps

      // Max. Current Limit for turning motor
      public static final int kTurningMotorCurrentLimit = 20;    // Amps

      // Wheel diameter in meters
      /*
      // For 3in MAAXSwerve Module, wheel diameter is 3 inches
      public stattic final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
       */

       // For SDS MK4i Module, wheel diameter is 4 inches
       public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);

      // Gear Motor Reduction
      /*
       * For MAX Swerve Module Base Kit Medium speed (13 teeth)
       * https://www.revrobotics.com/rev-21-3005/
       * 
       *               ┌──────────┬─────────────────────┬────────────┬────────────────┐
       *               │Gear Ratio│Free Speed (NEO V1.1)│Pinion Teeth│Spur Geaar Teeth│
       * ┌─────┬───────┼──────────┼─────────────────────┼────────────┼────────────────┤
       * │     │Low    │  5.50:1  │4.12 m/s (13.51 ft/s)│    12      │      22        │
       * │Base ├───────┼──────────┼─────────────────────┼────────────┼────────────────┤
       * │Kit  │Medium │  5.08:1  │4.46 m/s (14.63 ft/s)│    13      │      22        │
       * │     ├───────┼──────────┼─────────────────────┼────────────┼────────────────┤
       * │     │High   │  4.71:1  │4.80 m/s (15.76 ft/s)│    14      │      22        │
       * └─────┴───────┴──────────┴─────────────────────┴────────────┴────────────────┘
       *
      // 45 teeth on the wheel's bevel gear, 22 teeth on the 1st stage spur gear, 15 teeth on the bevel pinion
      public static final double kDrivingMotorReduction = (45.0 * 22.0) / (13.0 * 15.0);      // 5.08
      */

      /* For SDS Mk4i L2 Module
       * https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
       * 
       * ┌───────┬───────────────────────┬───────────────────────┬───────────────────────┐
       * │       │          L1 Ratio     │          L2 Ratio     │          L3 Ratio     │
       * │       ├────────────┬──────────┼───────────────────────┼───────────────────────┤
       * │       │Driving Gear│Drive Gear│Driving Gear│Drive Gear│Driving Gear│Drive Gear│
       * │ Stage │  (teeth)   │ (teeth)  │  (teeth)   │ (teeth)  │  (teeth)   │ (teeth)  │
       * ├───────┼────────────┼──────────┼────────────┼──────────┼────────────┼──────────┤
       * │  1st  │     14     │    50    │     14     │    50    │     14     │    50    │
       * ├───────┼────────────┼──────────┼────────────┼──────────┼────────────┼──────────┤
       * │  2nd  │     25     │    19    │     27     │    17    │     28     │    16    │
       * ├───────┼────────────┼──────────┼────────────┼──────────┼────────────┼──────────┤
       * │  3rd  │     15     │    45    │     15     │    45    │     15     │    45    │
       * ├───────┼────────────┴──────────┼────────────┴──────────┼────────────┴──────────┤
       * │Overall│          8.14:1       │          6.75:1       │          6.12:1       │
       * └───────┴───────────────────────┴───────────────────────┴───────────────────────┘
       * 
       */
      public static final double kDrivingMotorReduction = (50 / 14) * (17 / 27) * (45 / 15);    // 6.75

      public static final double kDrivingP = 0.04;
      public static final double kDrivingI = 0.00;
      public static final double kDrivingD = 0.00;

      public static final double kTurningP = 1.00;
      public static final double kTurningI = 0.00;
      public static final double kTurningD = 0.00;

      public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      // (5676 / 60) (4 * 0.0254 * 3.14) / 6.75 = 4.47 m/s
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

      // For MAX Swerve Module, the steering gear ratio is 12.0:1
      // public static final double kTurningMotorReduction = 12.0
      //
      // For SDS Mk4i, the steering gear ratio is 150/7:1
      public static final double kTurningMotorReduction = 150.0 / 7.0;

    }

    // NeoMotorConstants -----------------------------------------------------------------------------
    public static class NeoMotorConstants {
      public static final double kFreeSpeedRpm = 5676;      // RPM
    }

}
