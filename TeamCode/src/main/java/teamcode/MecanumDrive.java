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
import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class MecanumDrive extends RobotDrive
{
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public MecanumDrive(Robot robot)
    {
        super();

        lfDriveMotor = createDriveMotor(RobotParams.HWNAME_LFDRIVE_MOTOR, RobotParams.LEFT_WHEEL_INVERTED);
        lbDriveMotor = createDriveMotor(RobotParams.HWNAME_LBDRIVE_MOTOR, RobotParams.LEFT_WHEEL_INVERTED);
        rfDriveMotor = createDriveMotor(RobotParams.HWNAME_RFDRIVE_MOTOR, RobotParams.RIGHT_WHEEL_INVERTED);
        rbDriveMotor = createDriveMotor(RobotParams.HWNAME_RBDRIVE_MOTOR, RobotParams.RIGHT_WHEEL_INVERTED);

        driveBase = new TrcMecanumDriveBase(lfDriveMotor, lbDriveMotor, rfDriveMotor, rbDriveMotor, gyro);

        if (RobotParams.Preferences.useExternalOdometry)
        {
            //
            // Create the external odometry device that uses the left front encoder port as the X odometry and
            // the left and right back encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
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
            driveBase.setOdometryScales(RobotParams.XPOS_INCHES_PER_COUNT, RobotParams.Y_ODWHEEL_INCHES_PER_COUNT);
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
        purePursuitDrive.setMsgTracer(robot.globalTracer, true, true);
    }   //MecanumDrive

}   //class MecanumDrive
