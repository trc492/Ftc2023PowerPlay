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

import org.opencv.core.Point;

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcDriveBaseOdometry;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcFtcLib.ftclib.FtcBNO055Imu;
import TrcFtcLib.ftclib.FtcDcMotor;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class RobotDrive
{
    //
    // Sensors.
    //
    public final FtcBNO055Imu imu;
    public final TrcGyro gyro;
    //
    // Drive motors.
    //
    public final FtcDcMotor leftFrontWheel;
    public final FtcDcMotor rightFrontWheel;
    public final FtcDcMotor leftBackWheel;
    public final FtcDcMotor rightBackWheel;
    //
    // Drive Base.
    //
    public final TrcDriveBase driveBase;
    public final TrcPidController encoderXPidCtrl;
    public final TrcPidController encoderYPidCtrl;
    public final TrcPidController gyroPidCtrl;
    public final TrcPidDrive pidDrive;
    public final TrcPurePursuitDrive purePursuitDrive;
    //
    // Coefficients for PID controllers.
    //
    public final TrcPidController.PidCoefficients xPosPidCoeff;
    public final TrcPidController.PidCoefficients yPosPidCoeff;
    public final TrcPidController.PidCoefficients turnPidCoeff;
    public final TrcPidController.PidCoefficients velPidCoeff;

    /**
     * Constructor: Create an instance of the object.
     */
    public RobotDrive(Robot robot)
    {
        imu = new FtcBNO055Imu(RobotParams.HWNAME_IMU);
        gyro = imu.gyro;

        leftFrontWheel = new FtcDcMotor(RobotParams.HWNAME_LEFT_FRONT_WHEEL);
        rightFrontWheel = new FtcDcMotor(RobotParams.HWNAME_RIGHT_FRONT_WHEEL);
        leftBackWheel = new FtcDcMotor(RobotParams.HWNAME_LEFT_BACK_WHEEL);
        rightBackWheel = new FtcDcMotor(RobotParams.HWNAME_RIGHT_BACK_WHEEL);

        leftFrontWheel.motor.setMode(RobotParams.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotParams.DRIVE_MOTOR_MODE);
        leftBackWheel.motor.setMode(RobotParams.DRIVE_MOTOR_MODE);
        rightBackWheel.motor.setMode(RobotParams.DRIVE_MOTOR_MODE);

        if (RobotParams.Preferences.useVelocityControl)
        {
            leftFrontWheel.enableVelocityMode(RobotParams.DRIVE_MOTOR_MAX_VELOCITY_PPS);
            rightFrontWheel.enableVelocityMode(RobotParams.DRIVE_MOTOR_MAX_VELOCITY_PPS);
            leftBackWheel.enableVelocityMode(RobotParams.DRIVE_MOTOR_MAX_VELOCITY_PPS);
            rightBackWheel.enableVelocityMode(RobotParams.DRIVE_MOTOR_MAX_VELOCITY_PPS);
        }

        leftFrontWheel.setInverted(RobotParams.LEFT_WHEEL_INVERTED);
        leftBackWheel.setInverted(RobotParams.LEFT_WHEEL_INVERTED);
        rightFrontWheel.setInverted(RobotParams.RIGHT_WHEEL_INVERTED);
        rightBackWheel.setInverted(RobotParams.RIGHT_WHEEL_INVERTED);

        leftFrontWheel.setBrakeModeEnabled(RobotParams.DRIVE_WHEEL_BRAKE_MODE);
        leftBackWheel.setBrakeModeEnabled(RobotParams.DRIVE_WHEEL_BRAKE_MODE);
        rightFrontWheel.setBrakeModeEnabled(RobotParams.DRIVE_WHEEL_BRAKE_MODE);
        rightBackWheel.setBrakeModeEnabled(RobotParams.DRIVE_WHEEL_BRAKE_MODE);

        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, gyro);
        if (RobotParams.Preferences.useExternalOdometry)
        {
            //
            // Create the external odometry device that uses the left front encoder port as the X odometry and
            // the left and right back encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
            // odometry.
            //
            TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
                new TrcDriveBaseOdometry.AxisSensor(rightBackWheel, RobotParams.X_ODOMETRY_WHEEL_OFFSET),
                new TrcDriveBaseOdometry.AxisSensor[] {
                    new TrcDriveBaseOdometry.AxisSensor(leftFrontWheel, RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
                    new TrcDriveBaseOdometry.AxisSensor(rightFrontWheel, RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
                gyro);
            //
            // Set the drive base to use the external odometry device overriding the built-in one.
            //
            driveBase.setDriveBaseOdometry(driveBaseOdometry);
            driveBase.setOdometryScales(RobotParams.ODWHEEL_X_INCHES_PER_COUNT, RobotParams.ODWHEEL_Y_INCHES_PER_COUNT);
        }
        else
        {
            driveBase.setOdometryScales(RobotParams.ENCODER_X_INCHES_PER_COUNT, RobotParams.ENCODER_Y_INCHES_PER_COUNT);
        }
        //
        // Create and initialize PID controllers.
        //
        xPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ENCODER_X_KP, RobotParams.ENCODER_X_KI, RobotParams.ENCODER_X_KD);
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ENCODER_Y_KP, RobotParams.ENCODER_Y_KI, RobotParams.ENCODER_Y_KD);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_KP, RobotParams.GYRO_KI, RobotParams.GYRO_KD);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl", xPosPidCoeff, RobotParams.ENCODER_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl", yPosPidCoeff, RobotParams.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
            "gyroPidCtrl", turnPidCoeff, RobotParams.GYRO_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        // FTC robots generally have USB performance issues where the sampling rate of the gyro is not high enough.
        // If the robot turns too fast, PID will cause oscillation. By limiting turn power, the robot turns slower.
        gyroPidCtrl.setOutputLimit(RobotParams.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setBeep(robot.androidTone);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase, RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE,
            RobotParams.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setStallDetectionEnabled(true);
        pidDrive.setBeep(robot.androidTone);
    }   //RobotDrive

    /**
     * This method cancels any PIDDrive operation still in progress.
     */
    public void cancel()
    {
        if (pidDrive.isActive())
        {
            pidDrive.cancel();
        }

        if (purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel();
        }

        driveBase.stop();
    }   //cancel

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param xTargetLocation specifies the target location in field reference frame.
     * @param yTargetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @param tileUnit specifies true if location unit is in floor tile unit, false if in inches unit.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(double xTargetLocation, double yTargetLocation, double heading, boolean tileUnit)
    {
        double unitScale = tileUnit? RobotParams.FULL_TILE_INCHES: 1.0;

        return new TrcPose2D(xTargetLocation*unitScale, yTargetLocation*unitScale, heading);
    }   //pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param xTargetLocation specifies the target location in field reference frame.
     * @param yTargetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(double xTargetLocation, double yTargetLocation, double heading)
    {
        return pathPoint(xTargetLocation, yTargetLocation, heading, true);
    }   //pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param targetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @param tileUnit specifies true if location unit is in floor tile unit, false if in inches unit.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(Point targetLocation, double heading, boolean tileUnit)
    {
        return pathPoint(targetLocation.x, targetLocation.y, heading, tileUnit);
    }   //pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param targetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(Point targetLocation, double heading)
    {
        return pathPoint(targetLocation.x, targetLocation.y, heading, true);
    }   //pathPoint

}   //class RobotDrive
