/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package multiteams.team6541;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;

import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import multiteams.RobotDrive;
import multiteams.RobotParams;

/**
 * This class initializes the robot and subsystem constants and parameters.
 */
public class RobotParams6541
{
    public RobotParams6541()
    {
        // Preferences.
        RobotParams.Preferences.noRobot = false;
        RobotParams.Preferences.initSubsystems = true;
        RobotParams.Preferences.useExternalOdometry = false;
        RobotParams.Preferences.useBlinkin = true;
        RobotParams.Preferences.useVuforia = false;
        RobotParams.Preferences.showVuforiaView = false;
        RobotParams.Preferences.useTensorFlow = false;
        RobotParams.Preferences.showTensorFlowView = false;
        RobotParams.Preferences.useEasyOpenCV = false;
        RobotParams.Preferences.showEasyOpenCvView = false;
        RobotParams.Preferences.useTraceLog = true;
        RobotParams.Preferences.useBatteryMonitor = false;
        RobotParams.Preferences.useLoopPerformanceMonitor = true;
        RobotParams.Preferences.useVelocityControl = false;

        RobotParams.robotName = "Ftc6541";
        RobotParams.logPathFolder = Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc6541";
        //
        // Robot dimensions.
        //
        RobotParams.robotLength = 17.0;
        RobotParams.robotWidth = 13.0;
        RobotParams.driveBaseLength = 17.0;
        RobotParams.driveBaseWidth = 17.0;
        //
        // Game positions.
        //
        RobotParams.startPosRed1 = new TrcPose2D();
        RobotParams.startPosRed2 = new TrcPose2D();
        RobotParams.startPosBlue1 = new TrcPose2D();
        RobotParams.startPosBlue2 = new TrcPose2D();
        //
        // Vision subsystem.
        //
        RobotParams.cameraFrontOffset = 7.5;    //Camera offset from front of robot in inches
        RobotParams.cameraLeftOffset = 6.0;     //Camera offset from left of robot in inches
        RobotParams.cameraHeightOffset = 16.0;  //Camera offset from floor in inches
        // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
        // Measurement unit: inches
        RobotParams.worldRect = new TrcHomographyMapper.Rectangle(
            // topLeftX, topLeftY, topRightX, topRightY, bottomLeftX, bottomLeftY, bottomRightX, bottomRightY
            -22.25, 60.0, 23.0, 60.0, -8.75, 16.0, 7.5, 16.0);
        //
        // DriveBase subsystem.
        //
        RobotParams.driveMotorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        RobotParams.driveWheelBrakeModeOn = true;
        RobotParams.leftWheelInverted = true;
        RobotParams.rightWheelInverted = false;
        RobotParams.turnPowerLimit = 0.5;
        RobotParams.slowDrivePowerScale = 0.5;
        RobotParams.xOdometryWheelOffset = RobotParams.robotLength/2.0 + (3.875 + 9.5); // behind centroid
        RobotParams.yLeftOdometryWheelOffset = -15.25/2.0;
        RobotParams.yRightOdometryWheelOffset = 15.25/2.0;
        RobotParams.robotDriveMode = RobotDrive.DriveMode.ARCADE_MODE;

        RobotParams.xPosPidCoeff = new TrcPidController.PidCoefficients(0.02, 0.0, 0.0, 0.0);
        RobotParams.xPosTolerance = 2.0;
        RobotParams.xPosInchesPerCount = 9.072106867127145344367826764411e-4;
        // Unlike Mecanum drive, Swerve drive x and y PID parameters are identical.
        //noinspection SuspiciousNameCombination
        RobotParams.yPosPidCoeff = RobotParams.xPosPidCoeff;
        //noinspection SuspiciousNameCombination
        RobotParams.yPosTolerance = RobotParams.xPosTolerance;
        //noinspection SuspiciousNameCombination
        RobotParams.yPosInchesPerCount = RobotParams.xPosInchesPerCount;
        RobotParams.turnPidCoeff = new TrcPidController.PidCoefficients(0.013, 0.0, 0.0, 0.0);
        RobotParams.turnTolerance = 1.0;
        RobotParams.xOdometryWheelInchesPerCount = 7.6150160901199168116026724971383e-4;
        RobotParams.yOdometryWheelInchesPerCount = 7.6301149255006038191364659148717e-4;
        //
        // Pure Pursuit parameters.
        //
        RobotParams.robotMaxVelocity = 25.0;        // measured maximum from drive speed test.
        RobotParams.robotMaxAcceleration = 3380.0;  // measured maximum from drive speed test.
        // VelPid KF should be reciprocal of max tangential velocity (time to travel unit distance), units: sec/in.
        RobotParams.velPidCoeff = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1/RobotParams.robotMaxVelocity);
        RobotParams.ppdFollowingDistance = 6.0;
        RobotParams.ppdPosTolerance = 2.0;
        RobotParams.ppdTurnTolerance = 1.0;
        //
        // Subsystem parameters.
        //
    }   //RobotParams6541

}   //class RobotParams6541
