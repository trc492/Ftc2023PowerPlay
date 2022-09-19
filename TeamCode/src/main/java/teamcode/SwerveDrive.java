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

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Scanner;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDriveBaseOdometry;
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

    private static final String STEERING_CALIBRATION_DATA_FILE = "SteerCalibration.txt";
    public static final String[] servoNames = {
        RobotParams.HWNAME_LFSTEER_SERVO1, RobotParams.HWNAME_RFSTEER_SERVO1,
        RobotParams.HWNAME_LBSTEER_SERVO1, RobotParams.HWNAME_RBSTEER_SERVO1};
    public double[][] servoPositions = {
        {(RobotParams.LFSTEER_MINUS90 + RobotParams.LFSTEER_PLUS90)/2.0,
            RobotParams.LFSTEER_PLUS90, RobotParams.LFSTEER_MINUS90},
        {(RobotParams.RFSTEER_MINUS90 + RobotParams.RFSTEER_PLUS90)/2.0,
            RobotParams.RFSTEER_PLUS90, RobotParams.RFSTEER_MINUS90},
        {(RobotParams.LBSTEER_MINUS90 + RobotParams.LBSTEER_PLUS90)/2.0,
            RobotParams.LBSTEER_PLUS90, RobotParams.LBSTEER_MINUS90},
        {(RobotParams.RBSTEER_MINUS90 + RobotParams.RBSTEER_PLUS90)/2.0,
            RobotParams.RBSTEER_PLUS90, RobotParams.RBSTEER_MINUS90}
    };
    //
    // Swerve steering motors and modules.
    //
    public final FtcServo lfSteerServo1, lfSteerServo2, lbSteerServo1, lbSteerServo2;
    public final FtcServo rfSteerServo1, rfSteerServo2, rbSteerServo1, rbSteerServo2;
    public final TrcSwerveModule lfSwerveModule, lbSwerveModule, rfSwerveModule, rbSwerveModule;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public SwerveDrive(Robot robot)
    {
        super();

        lfDriveMotor = createDriveMotor(RobotParams.HWNAME_LFDRIVE_MOTOR, RobotParams.LFDRIVE_INVERTED);
        lbDriveMotor = createDriveMotor(RobotParams.HWNAME_LBDRIVE_MOTOR, RobotParams.LBDRIVE_INVERTED);
        rfDriveMotor = createDriveMotor(RobotParams.HWNAME_RFDRIVE_MOTOR, RobotParams.RFDRIVE_INVERTED);
        rbDriveMotor = createDriveMotor(RobotParams.HWNAME_RBDRIVE_MOTOR, RobotParams.RBDRIVE_INVERTED);

        lfSteerServo1 = createSteerServo(
            RobotParams.HWNAME_LFSTEER_SERVO1, servoPositions[0][0], servoPositions[0][1],
            RobotParams.LFSTEER_INVERTED);
        lfSteerServo2 = createSteerServo(
            RobotParams.HWNAME_LFSTEER_SERVO2, servoPositions[0][0], servoPositions[0][1],
            RobotParams.LFSTEER_INVERTED);
        lfSteerServo1.addFollower(lfSteerServo2);

        lbSteerServo1 = createSteerServo(
            RobotParams.HWNAME_LBSTEER_SERVO1, servoPositions[1][0], servoPositions[1][1],
            RobotParams.LBSTEER_INVERTED);
        lbSteerServo2 = createSteerServo(
            RobotParams.HWNAME_LBSTEER_SERVO2, servoPositions[1][0], servoPositions[1][1],
            RobotParams.LBSTEER_INVERTED);
        lbSteerServo1.addFollower(lbSteerServo2);

        rfSteerServo1 = createSteerServo(
            RobotParams.HWNAME_RFSTEER_SERVO1, servoPositions[2][0], servoPositions[2][1],
            RobotParams.RFSTEER_INVERTED);
        rfSteerServo2 = createSteerServo(
            RobotParams.HWNAME_RFSTEER_SERVO2, servoPositions[2][0], servoPositions[2][1],
            RobotParams.RFSTEER_INVERTED);
        rfSteerServo1.addFollower(rfSteerServo2);

        rbSteerServo1 = createSteerServo(
            RobotParams.HWNAME_RBSTEER_SERVO1, servoPositions[3][0], servoPositions[3][1],
            RobotParams.RBSTEER_INVERTED);
        rbSteerServo2 = createSteerServo(
            RobotParams.HWNAME_RBSTEER_SERVO2, servoPositions[3][0], servoPositions[3][1],
            RobotParams.RBSTEER_INVERTED);
        rbSteerServo1.addFollower(rbSteerServo2);

        lfSwerveModule = new TrcSwerveModule("lfSwerveModule", lfDriveMotor, lfSteerServo1);
        lbSwerveModule = new TrcSwerveModule("lbSwerveModule", lbDriveMotor, lbSteerServo1);
        rfSwerveModule = new TrcSwerveModule("rfSwerveModule", rfDriveMotor, rfSteerServo1);
        rbSwerveModule = new TrcSwerveModule("rbSwerveModule", rbDriveMotor, rbSteerServo1);
        lfSwerveModule.setSteeringLimits(RobotParams.STEER_LOW_LIMIT, RobotParams.STEER_HIGH_LIMIT);
        lbSwerveModule.setSteeringLimits(RobotParams.STEER_LOW_LIMIT, RobotParams.STEER_HIGH_LIMIT);
        rfSwerveModule.setSteeringLimits(RobotParams.STEER_LOW_LIMIT, RobotParams.STEER_HIGH_LIMIT);
        rbSwerveModule.setSteeringLimits(RobotParams.STEER_LOW_LIMIT, RobotParams.STEER_HIGH_LIMIT);

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
             driveBase.setOdometryScales(RobotParams.YPOS_INCHES_PER_COUNT, RobotParams.YPOS_INCHES_PER_COUNT);
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
     * This method creates and configures a steering servo.
     *
     * @param servoName specifies the name of the servo.
     * @param steerMinus90 specifies the logical position of -90 degree.
     * @param steerPlus90 specifies the logical position of +90 degree.
     * @param inverted specifies true if servo direction is reversed, false otherwise.
     * @return created steering servo.
     */
    private FtcServo createSteerServo(String servoName, double steerMinus90, double steerPlus90, boolean inverted)
    {
        FtcServo servo = new FtcServo(servoName);

        servo.setInverted(inverted);
        servo.setPhysicalRange(-90.0, 90.0);
        servo.setLogicalRange(steerMinus90, steerPlus90);

        return servo;
    }   //createSteerServo

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

    /**
     * This method sets all the swerve steering servos to the selected angle.
     *
     * @param index specifies the index in the servo position table.
     */
    public void setSteeringServoPosition(int index)
    {
        lfSteerServo1.setLogicalPosition(servoPositions[0][index]);
        lfSteerServo2.setLogicalPosition(servoPositions[0][index]);
        rfSteerServo1.setLogicalPosition(servoPositions[1][index]);
        rfSteerServo2.setLogicalPosition(servoPositions[1][index]);
        lbSteerServo1.setLogicalPosition(servoPositions[2][index]);
        lbSteerServo2.setLogicalPosition(servoPositions[2][index]);
        rbSteerServo1.setLogicalPosition(servoPositions[3][index]);
        rbSteerServo2.setLogicalPosition(servoPositions[3][index]);
    }  //setSteeringServoPosition

    /**
     * This method saves the calibration data to a file on the Robot Controller.
     */
    public void saveSteeringCalibrationData()
    {
        final String funcName = "saveSteeringCalibrationData";

        try (PrintStream out = new PrintStream(new FileOutputStream(
            RobotParams.TEAM_FOLDER_PATH + "/" + STEERING_CALIBRATION_DATA_FILE)))
        {
            for (int i = 0; i < servoNames.length; i++)
            {
                out.printf("%s: %f, %f\n", servoNames[i], servoPositions[i][0], servoPositions[i][1]);
            }
            out.close();
            TrcDbgTrace.getGlobalTracer().traceInfo(funcName, "Saved steering calibration data!");
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveSteeringCalibrationData

    /**
     * This method reads the steering calibration data from a file on the Robot Controller.
     *
     * @throws RuntimeException if file contains invalid data.
     */
    public void readSteeringCalibrationData()
    {
        final String funcName = "readSteeringCalibrationData";

        try (Scanner in = new Scanner(new FileReader(
            RobotParams.TEAM_FOLDER_PATH + "/" + STEERING_CALIBRATION_DATA_FILE)))
        {
            for (int i = 0; i < servoNames.length; i++)
            {
                String line = in.nextLine();
                int colonPos = line.indexOf(':');
                String name = colonPos == -1? null: line.substring(0, colonPos);

                if (name == null || !name.equals(servoNames[i]))
                {
                    throw new RuntimeException("Invalid servo name in line " + line);
                }

                String[] numbers = line.substring(colonPos + 1).split(",", 2);

                for (int j = 0; j < servoPositions[0].length; j++)
                {
                    servoPositions[i][j] = Double.parseDouble(numbers[j]);
                }
            }
        }
        catch (FileNotFoundException e)
        {
            TrcDbgTrace.getGlobalTracer().traceWarn(
                funcName, "Steering calibration data file not found, using built-in defaults.");
        }
    }   //readSteeringCalibrationData

}   //class SwerveDrive
