/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

#include "Robot.h"




Robot::Robot() {
    // Note SmartDashboard is not initialized here, wait until RobotInit() to make
    // SmartDashboard calls
    drive.SetExpiration(0.1);
}

void Robot::RobotInit() {
    frc::CameraServer::GetInstance()->StartAutomaticCapture();

    frontLeft.ConfigNeutralDeadband(deadbandPercent, 0);
    frontRight.ConfigNeutralDeadband(deadbandPercent, 0);
    backLeft.ConfigNeutralDeadband(deadbandPercent, 0);
    backRight.ConfigNeutralDeadband(deadbandPercent, 0);

    sensorPrep();
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::Autonomous() {
    std::string autoSelected = m_chooser.GetSelected();
    // std::string autoSelected = frc::SmartDashboard::GetString(
    // "Auto Selector", kAutoNameDefault);
    std::cout << "Auto selected: " << autoSelected << std::endl;

    // MotorSafety improves safety when motors are updated in loops but is
    // disabled here because motor updates are not looped in this autonomous mode.
    //m_robotDrive.SetSafetyEnabled(false);

    if (autoSelected == kAutoNameCustom) {
        // Custom Auto goes here
        std::cout << "Running custom Autonomous" << std::endl;

        // Spin at half speed for two seconds
        //m_robotDrive.ArcadeDrive(0.0, 0.5);
        frc::Wait(2.0);

        // Stop robot
        //m_robotDrive.ArcadeDrive(0.0, 0.0);
    } else {
        // Default Auto goes here
        std::cout << "Running default Autonomous" << std::endl;

        // Drive forwards at half speed for two seconds
        //m_robotDrive.ArcadeDrive(-0.5, 0.0);
        frc::Wait(2.0);

        // Stop robot
        //m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
}

/**
 * Runs the motors with arcade steering.
 */
void Robot::OperatorControl() {
    drive.SetSafetyEnabled(true);
    barTwo.SetSelectedSensorPosition(0, 0, 0);
    elevatorTwo.SetSelectedSensorPosition(0, 0, 0);

    double yAxis;
    double xAxis;
    double zAxis;
    double elevatorSpeed;
    double barSpeed;
    
    while (IsOperatorControl() && IsEnabled()) 
    {//2500,7000,10100.9200

        xAxis = pow(controller.GetX(leftHand), axisExponent);
        yAxis = pow(controller.GetY(leftHand), axisExponent);
        zAxis = pow(controller.GetX(rightHand), axisExponent);
        drive.DriveCartesian(xAxis, -yAxis, zAxis);

        barSpeed = pow(controller.GetY(rightHand), axisExponent);
        barSpeed -= .15;
        barSpeed = (barSpeed > powerMax) ? powerMax : barSpeed;
        bar.Set(-barSpeed);
        
        if(controller.GetBumper(leftHand) == 1)
        {
            intake.Set(-.5);
        }
        else if(controller.GetBumper(rightHand) == 1)
        {
            intake.Set(.5);
        }
        else
        {
            intake.Set(0);
        }

        if(controller.GetTriggerAxis(leftHand) > controller.GetTriggerAxis(rightHand))
        {
            elevatorSpeed = pow(controller.GetTriggerAxis(leftHand), axisExponent);
            elevatorSpeed += .1;
            elevatorSpeed = (elevatorSpeed > powerMax) ? powerMax : elevatorSpeed;
            elevator.Set(-elevatorSpeed);
        }
        else
        {
            elevatorSpeed = pow(controller.GetTriggerAxis(rightHand), axisExponent);
            elevatorSpeed += .1;
            elevatorSpeed = (elevatorSpeed > powerMax) ? powerMax : elevatorSpeed;
            elevator.Set(elevatorSpeed);
        }
        if(controller.GetAButton())
        {
            barTwo.Set(ControlMode::MotionMagic, 2500);
            barOne.Follow(barTwo);
        }
        if(controller.GetXButton())
        {
            barTwo.Set(ControlMode::MotionMagic, 7100);
            barOne.Follow(barTwo);
        }
        if(controller.GetYButton())
        {
            barTwo.Set(ControlMode::MotionMagic, 9500);
            barOne.Follow(barTwo);
        }
        // The motors will be updated every 5ms
        dashboardDiagnostic();
        std::cout << "LD:" << backLeft.GetSelectedSensorPosition() << ",RD:" << backRight.GetSelectedSensorPosition() << ",B:" << barTwo.GetSelectedSensorPosition() << ",E:" << elevatorTwo.GetSelectedSensorPosition() << std::endl;  
        frc::Wait(0.005);
    }
}

void Robot::low()
{

}
void Robot::mid()
{

}
void Robot::high()
{

}
void Robot::dashboardDiagnostic()
{    
    //Bools
    frc::SmartDashboard::PutBoolean("FL", power.GetCurrent(14) != 0);
    frc::SmartDashboard::PutBoolean("FR", power.GetCurrent(15) != 0);
    frc::SmartDashboard::PutBoolean("BL", power.GetCurrent(12) != 0);
    frc::SmartDashboard::PutBoolean("BR", power.GetCurrent(13) != 0);

    frc::SmartDashboard::PutBoolean("EL1", power.GetCurrent(2) != 0);
    frc::SmartDashboard::PutBoolean("EL2", power.GetCurrent(3) != 0);

    frc::SmartDashboard::PutBoolean("B1", power.GetCurrent(0) != 0);
    frc::SmartDashboard::PutBoolean("B2", power.GetCurrent(1) != 0);

    frc::SmartDashboard::PutBoolean("Intake", power.GetCurrent(4) != 0);
    
    frc::SmartDashboard::PutBoolean("Total", (power.GetTotalCurrent() < 100));

    //Numbers
    frc::SmartDashboard::PutNumber("FL-num", power.GetCurrent(14));
    frc::SmartDashboard::PutNumber("FR-num", power.GetCurrent(15));
    frc::SmartDashboard::PutNumber("BL-num", power.GetCurrent(12));
    frc::SmartDashboard::PutNumber("BR-num", power.GetCurrent(13));

    frc::SmartDashboard::PutNumber("EL1-num", power.GetCurrent(2));
    frc::SmartDashboard::PutNumber("EL2-num", power.GetCurrent(3));

    frc::SmartDashboard::PutNumber("B1-num", power.GetCurrent(0));
    frc::SmartDashboard::PutNumber("B2-num", power.GetCurrent(1));

    frc::SmartDashboard::PutNumber("Intake-num", power.GetCurrent(4));
    
    frc::SmartDashboard::PutNumber("Total-num", power.GetTotalCurrent());

}
void Robot::sensorPrep()
{
    backLeft.ConfigSelectedFeedbackSensor(pulseWidth, 0, 0);
    backRight.ConfigSelectedFeedbackSensor(pulseWidth, 0, 0);
    elevatorTwo.ConfigSelectedFeedbackSensor(pulseWidth, 0, 0);
    barTwo.ConfigSelectedFeedbackSensor(pulseWidth, 0, 0);

    backLeft.ConfigSelectedFeedbackCoefficient(driveEncoderScaler, 0, 0);
    backRight.ConfigSelectedFeedbackCoefficient(driveEncoderScaler, 0, 0);
    elevatorTwo.ConfigSelectedFeedbackCoefficient(1, 0, 0);
    barTwo.ConfigSelectedFeedbackCoefficient(1, 0, 0);

    barTwo.SetSensorPhase(true);
    barOne.SetInverted(true);
    barTwo.SetInverted(true);
    barTwo.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    barTwo.ConfigNominalOutputForward(.25, 10);
    barTwo.ConfigNominalOutputReverse(0, 10);
    barTwo.ConfigPeakOutputForward(1, 10);
    barTwo.ConfigPeakOutputReverse(-1, 10);

    barTwo.SelectProfileSlot(0, 0);
    barTwo.Config_kF(0, 1, 10);
    barTwo.Config_kP(0, 0.05, 10);
    barTwo.Config_kI(0, 0, 10);
    barTwo.Config_kD(0, 0, 10);

    barTwo.ConfigMotionCruiseVelocity(850, 10);
    barTwo.ConfigMotionAcceleration(1500, 10);
    //barTwo.ConfigAllowableClosedloopError(0, 100, 0);

    elevatorTwo.SetSensorPhase(true);
    elevatorTwo.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);

    elevatorTwo.ConfigNominalOutputForward(.2, 10);
    elevatorTwo.ConfigNominalOutputReverse(0, 10);
    elevatorTwo.ConfigPeakOutputForward(1, 10);
    elevatorTwo.ConfigPeakOutputReverse(-1, 10);

    elevatorTwo.SelectProfileSlot(0, 0);
    elevatorTwo.Config_kF(0, 1, 10);
    elevatorTwo.Config_kP(0, 0.5, 10);
    elevatorTwo.Config_kI(0, 0, 10);
    elevatorTwo.Config_kD(0, 0, 10);

    elevatorTwo.ConfigMotionCruiseVelocity(5000, 10);
    elevatorTwo.ConfigMotionAcceleration(3000, 10);

    backLeft.SetSelectedSensorPosition(0, 0, 0);
    backRight.SetSelectedSensorPosition(0, 0, 0);
    elevatorTwo.SetSelectedSensorPosition(0, 0, 0);
    barTwo.SetSelectedSensorPosition(0, 0, 0);

}
/**
 * Runs during test mode
 */
void Robot::Test() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
