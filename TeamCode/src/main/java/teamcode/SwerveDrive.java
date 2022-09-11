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

package teamcode;

import TrcCommonLib.trclib.TrcDriveBaseOdometry;
import TrcCommonLib.trclib.TrcEnhancedServo;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
import TrcFtcLib.ftclib.FtcServo;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class SwerveDrive extends RobotDrive
{
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;
    //
    // Swerve steering motors and modules.
    //
    public final FtcServo lfSteerServo1, lfSteerServo2, lbSteerServo1, lbSteerServo2;
    public final FtcServo rfSteerServo1, rfSteerServo2, rbSteerServo1, rbSteerServo2;
    public final TrcEnhancedServo lfSteerServo, lbSteerServo, rfSteerServo, rbSteerServo;
    public final TrcSwerveModule lfSwerveModule, lbSwerveModule, rfSwerveModule, rbSwerveModule;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public SwerveDrive(Robot robot)
    {
        super();

        lfDriveMotor = createDriveMotor(RobotParams.HWNAME_LFDRIVE_MOTOR, false);
        lbDriveMotor = createDriveMotor(RobotParams.HWNAME_LBDRIVE_MOTOR, false);
        rfDriveMotor = createDriveMotor(RobotParams.HWNAME_RFDRIVE_MOTOR, false);
        rbDriveMotor = createDriveMotor(RobotParams.HWNAME_RBDRIVE_MOTOR, false);

        lfSteerServo1 = new FtcServo(RobotParams.HWNAME_LFSTEER_SERVO1);
        lfSteerServo2 = new FtcServo(RobotParams.HWNAME_LFSTEER_SERVO2);
        lbSteerServo1 = new FtcServo(RobotParams.HWNAME_LBSTEER_SERVO1);
        lbSteerServo2 = new FtcServo(RobotParams.HWNAME_LBSTEER_SERVO2);
        rfSteerServo1 = new FtcServo(RobotParams.HWNAME_RFSTEER_SERVO1);
        rfSteerServo2 = new FtcServo(RobotParams.HWNAME_RFSTEER_SERVO2);
        rbSteerServo1 = new FtcServo(RobotParams.HWNAME_RBSTEER_SERVO1);
        rbSteerServo2 = new FtcServo(RobotParams.HWNAME_RBSTEER_SERVO2);

        lfSteerServo = new TrcEnhancedServo("lfSteerServo", lfSteerServo1, lfSteerServo2);
        lbSteerServo = new TrcEnhancedServo("lbSteerServo", lbSteerServo1, lbSteerServo2);
        rfSteerServo = new TrcEnhancedServo("rfSteerServo", rfSteerServo1, rfSteerServo2);
        rbSteerServo = new TrcEnhancedServo("rbSteerServo", rbSteerServo1, rbSteerServo2);

        lfSwerveModule = new TrcSwerveModule("lfSwerveModule", lfDriveMotor, lfSteerServo);
        lbSwerveModule = new TrcSwerveModule("lbSwerveModule", lbDriveMotor, lbSteerServo);
        rfSwerveModule = new TrcSwerveModule("rfSwerveModule", rfDriveMotor, rfSteerServo);
        rbSwerveModule = new TrcSwerveModule("rbSwerveModule", rbDriveMotor, rbSteerServo);

        driveBase = new TrcSwerveDriveBase(
            lfSwerveModule, lbSwerveModule, rfSwerveModule, rbSwerveModule, gyro,
            RobotParams.DRIVE_BASE_WIDTH, RobotParams.DRIVE_BASE_LENGTH);
        driveBase.setSynchronizeOdometriesEnabled(false);

         if (RobotParams.Preferences.useExternalOdometry)
         {
             //
             // Create the external odometry device that uses the right back encoder port as the X odometry and
             // the left and right front encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
             // odometry.
             //
             TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
                 new TrcDriveBaseOdometry.AxisSensor(rbDriveMotor, RobotParams.X_ODOMETRY_WHEEL_OFFSET),
                 new TrcDriveBaseOdometry.AxisSensor[] {
                     new TrcDriveBaseOdometry.AxisSensor(lfDriveMotor, RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
                     new TrcDriveBaseOdometry.AxisSensor(rfDriveMotor, RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
                 gyro);
             //
             // Set the drive base to use the external odometry device overriding the built-in one.
             //
             driveBase.setDriveBaseOdometry(driveBaseOdometry);
             driveBase.setOdometryScales(
                 RobotParams.X_ODWHEEL_INCHES_PER_COUNT, RobotParams.Y_ODWHEEL_INCHES_PER_COUNT);
         }
         else
         {
             driveBase.setOdometryScales(RobotParams.XPOS_INCHES_PER_COUNT, RobotParams.YPOS_INCHES_PER_COUNT);
         }

        //
        // Create and initialize PID controllers.
        //
        xPosPidCtrl = new TrcPidController(
            "xPosPidCtrl", RobotParams.xPosPidCoeff, RobotParams.XPOS_TOLERANCE, driveBase::getXPosition);
        yPosPidCtrl = new TrcPidController(
            "yPosPidCtrl", RobotParams.yPosPidCoeff, RobotParams.YPOS_TOLERANCE, driveBase::getYPosition);
        turnPidCtrl = new TrcPidController(
            "turnPidCtrl", RobotParams.turnPidCoeff, RobotParams.TURN_TOLERANCE, driveBase::getHeading);
        turnPidCtrl.setAbsoluteSetPoint(true);
        // FTC robots generally have USB performance issues where the sampling rate of the gyro is not high enough.
        // If the robot turns too fast, PID will cause oscillation. By limiting turn power, the robot turns slower.
        turnPidCtrl.setOutputLimit(RobotParams.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, xPosPidCtrl, yPosPidCtrl, turnPidCtrl);
        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE, RobotParams.PPD_TURN_TOLERANCE,
            RobotParams.xPosPidCoeff, RobotParams.yPosPidCoeff, RobotParams.turnPidCoeff, RobotParams.velPidCoeff);
        purePursuitDrive.setStallDetectionEnabled(true);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);
    }   //SwerveDrive

    /**
     * This method checks if anti-defense mode is enabled.
     *
     * @return true if anti-defense mode is enabled, false if disabled.
     */
    public boolean isAntiDefenseEnabled()
    {
        return ((TrcSwerveDriveBase) driveBase).isAntiDefenseEnabled();
    }   //isAntiDefenseEnabled

    /**
     * This method enables/disables the anti-defense mode where it puts all swerve wheels into an X-formation.
     * By doing so, it is very difficult for others to push us around.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param enabled   specifies true to enable anti-defense mode, false to disable.
     */
    public void setAntiDefenseEnabled(String owner, boolean enabled)
    {
        if (owner == null || !enabled || driveBase.acquireExclusiveAccess(owner))
        {
            ((TrcSwerveDriveBase) driveBase).setAntiDefenseEnabled(owner, enabled);
            if (!enabled)
            {
                driveBase.releaseExclusiveAccess(owner);
            }
        }
    }   //setAntiDefenseEnabled

}   //class SwerveDrive
