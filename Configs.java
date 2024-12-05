// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class Configs {
    public static final class MAXSwerveModule {
        // Objects
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        // Variables
        static {
            // Factor to convert from Rotations to meters
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
            // Factor to convert from Rotations to radians
            double turningFactor = 2.0 * Math.PI / ModuleConstants.kTurningMotorReduction;

            // Configure driving motor
            drivingConfig
                // set to Brake mode when driving motor is in idle
                .idleMode(IdleMode.kBrake)
                // set max. current limit to driving motor
                .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

            // Configure driving encoder
            drivingConfig.encoder
                // set positionConversionFactor for driving motor
                .positionConversionFactor(drivingFactor)
                // set velocityConversionFactor for driving motor
                .velocityConversionFactor(drivingFactor / 60.0);

            // Configure PID Closed Loop Controller for driving motor
            drivingConfig.closedLoop
                // tell PID Controller to use primary encoder (Relative encoder) as feedback device
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // set PID values. Replace these PID gains after running SysId tests
                .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                // Set the velocity feedforward gain of the controller.
                .velocityFF(1 / ModuleConstants.kDriveWheelFreeSpeedRps)
                // Set the output range of the controller.
                .outputRange(-1, 1);

            
            // Configure turning motor
            turningConfig
                // set to Brake mode when turning motor is in idle
                .idleMode(IdleMode.kBrake)
                // set velocityConversionFactor for turning motor
                .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

            turningConfig.absoluteEncoder
                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering motor in the MAXSwerve Module.
                .inverted(true)
                // set positionConversionFactor for turning motor
                .positionConversionFactor(turningFactor)
                // set velocityConversionFactor for turning motor
                .velocityConversionFactor(turningFactor / 60);

            turningConfig.closedLoop
                // // tell PID Controller to use secondarey encoder (Absolute encoder) as feedback device
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // set PID values. Replace these PID gains after running SysId tests
                .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
                // Set the output range of the controller.
                .outputRange(-1, 1)
                // Enable PID wrap around for turning motor. This will allow PID controller
                // to go through 0 to get the setpoint i.e. going from 360 deggrees
                // to 10 degrees will go through 0 rqther than order direction which is a longer route,
                .positionWrappingEnabled(true)
                // set the input range for PID closedLoop control
                .positionWrappingInputRange(0, 2.0 * Math.PI);
        }
    }
}
