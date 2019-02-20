/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <iostream>
#include <string>
#include <math.h>

#include "ctre/Phoenix.h"

#include <frc/WPILib.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SampleRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use TimedRobot or Command-Based
 * instead if you're new.
 */
class Robot : public frc::SampleRobot {
    public:
    Robot();

    void RobotInit() override;
    void Autonomous() override;
    void OperatorControl() override;
    void Test() override;

    void low();
    void mid();
    void high();

    void dashboardDiagnostic();
    void sensorPrep();

    private:
    // Robot drive system
    WPI_VictorSPX frontLeft{10};
    WPI_VictorSPX frontRight{9};
    WPI_TalonSRX backLeft{7};
    WPI_TalonSRX backRight{8};
    WPI_TalonSRX elevatorOne{5};
    WPI_TalonSRX elevatorTwo{6};
    WPI_TalonSRX barOne{3};
    WPI_TalonSRX barTwo{4};
    frc::Spark intake{0};

    frc::SpeedControllerGroup elevator{elevatorOne, elevatorTwo};
    frc::SpeedControllerGroup bar{barOne, barTwo};
    
    frc::MecanumDrive drive{frontLeft, backLeft, frontRight, backRight};

    const frc::GenericHID::JoystickHand leftHand = frc::GenericHID::JoystickHand::kLeftHand;
    const frc::GenericHID::JoystickHand rightHand = frc::GenericHID::JoystickHand::kRightHand;

    const ctre::phoenix::motorcontrol::FeedbackDevice pulseWidth = ctre::phoenix::motorcontrol::FeedbackDevice::PulseWidthEncodedPosition;
    frc::XboxController controller{0};

    frc::PowerDistributionPanel power{1};

    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";

    const double deadbandPercent = .0005;//try 1? i think the "deadzones" are really just a
                                         //large area with too little power to move at all
    const double axisExponent = 3;
    const double powerMax = .8;
    const double powerScaler = .35;
    const double driveEncoderScaler = 0.0061359232;
    const double elevatorEncoderScaler = 0.0010983302;
    const double barEncoderScaler = 0.009765625;
};
