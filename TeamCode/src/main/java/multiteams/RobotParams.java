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

package multiteams;

import com.qualcomm.robotcore.hardware.DcMotor;

import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains robot preferences. It controls enabling/disabling of various robot features.
     */
    public static class Preferences
    {
        public static boolean noRobot = false;
        public static boolean initSubsystems = true;
        public static boolean useExternalOdometry = false;
        public static boolean useBlinkin = false;
        public static boolean useVuforia = false;
        public static boolean showVuforiaView = false;
        public static boolean useTensorFlow = false;
        public static boolean showTensorFlowView = false;
        public static boolean useEasyOpenCV = false;
        public static boolean showEasyOpenCvView = false;
        public static boolean useTraceLog = true;
        public static boolean useBatteryMonitor = false;
        public static boolean useLoopPerformanceMonitor = true;
        public static boolean useVelocityControl = false;
    }   //class Preferences

    public static String robotName;
    public static String logPathFolder;

    //
    // Hardware names.
    //
    public static final String HWNAME_IMU                       = "imu";
    public static final String HWNAME_WEBCAM                    = "Webcam 1";
    public static final String HWNAME_BLINKIN                   = "blinkin";
    public static final String HWNAME_LFDRIVE_MOTOR             = "lfDriveMotor";
    public static final String HWNAME_RFDRIVE_MOTOR             = "rfDriveMotor";
    public static final String HWNAME_LBDRIVE_MOTOR             = "lbDriveMotor";
    public static final String HWNAME_RBDRIVE_MOTOR             = "rbDriveMotor";
    public static final String HWNAME_LFSTEER_SERVO1            = "lfSteerServo1";
    public static final String HWNAME_LFSTEER_SERVO2            = "lfSteerServo2";
    public static final String HWNAME_RFSTEER_SERVO1            = "rfSteerServo1";
    public static final String HWNAME_RFSTEER_SERVO2            = "rfSteerServo2";
    public static final String HWNAME_LBSTEER_SERVO1            = "lbSteerServo1";
    public static final String HWNAME_LBSTEER_SERVO2            = "lbSteerServo2";
    public static final String HWNAME_RBSTEER_SERVO1            = "rbSteerServo1";
    public static final String HWNAME_RBSTEER_SERVO2            = "rbSteerServo2";
    //
    // Field dimensions.
    //
    public static final double FULL_FIELD_INCHES                = 141.0;
    public static final double HALF_FIELD_INCHES                = FULL_FIELD_INCHES/2.0;
    public static final double FULL_TILE_INCHES                 = 23.75;
    public static final double HALF_TILE_INCHES                 = FULL_TILE_INCHES/2.0;
    //
    // Robot dimensions.
    //
    public static double robotLength;
    public static double robotWidth;
    public static double driveBaseLength;
    public static double driveBaseWidth;
    //
    // Game positions.
    //
    public static TrcPose2D startPosRed1;
    public static TrcPose2D startPosRed2;
    public static TrcPose2D startPosBlue1;
    public static TrcPose2D startPosBlue2;
    //
    // Motor Odometries.
    //
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_312_ENCODER_PPR     = ((((1.0 + (46.0/17.0)))*(1.0 + (46.0/11.0)))*28.0);
    public static final double GOBILDA_5203_312_RPM             = 312.0;
    public static final double GOBILDA_5203_312_MAX_VELOCITY_PPS=
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // 2795.987 pps
    //
    // DriveBase subsystem.
    //
    public static DcMotor.RunMode driveMotorMode;
    public static boolean driveWheelBrakeModeOn;
    public static boolean leftWheelInverted;
    public static boolean rightWheelInverted;
    public static double turnPowerLimit;
    public static double slowDrivePowerScale;
    public static double xOdometryWheelOffset;
    public static double yLeftOdometryWheelOffset;
    public static double yRightOdometryWheelOffset;
    public static RobotDrive.DriveMode robotDriveMode;
    //
    // Velocity controlled constants.
    //
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS     = GOBILDA_5203_312_MAX_VELOCITY_PPS;

    public static TrcPidController.PidCoefficients xPosPidCoeff;
    public static double xPosTolerance;
    public static double xPosInchesPerCount;

    public static TrcPidController.PidCoefficients yPosPidCoeff;
    public static double yPosTolerance;
    public static double yPosInchesPerCount;

    public static TrcPidController.PidCoefficients turnPidCoeff;
    public static double turnTolerance;

    public static double xOdometryWheelInchesPerCount;
    public static double yOdometryWheelInchesPerCount;
    //
    // Pure Pursuit parameters.
    //
    // No-Load max velocity (i.e. theoretical maximum)
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    public static double robotMaxVelocity;
    public static double robotMaxAcceleration;
    public static TrcPidController.PidCoefficients velPidCoeff;
    public static double ppdFollowingDistance;
    public static double ppdPosTolerance;
    public static double ppdTurnTolerance;
    //
    // Vision subsystem.
    //
    public static final String TRACKABLE_IMAGES_FILE            = "TrackableImagesFileName";
    public static double cameraFrontOffset;
    public static double cameraLeftOffset;
    public static double cameraHeightOffset;
    public static final int CAMERA_IMAGE_WIDTH                  = 320;
    public static final int CAMERA_IMAGE_HEIGHT                 = 240;
    public static final int FRAME_QUEUE_CAPACITY                = 2;
    public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
        // topLeftX, topLeftY, topRightX, topRightY
        0.0, 0.0, CAMERA_IMAGE_WIDTH - 1, 0.0,
        // bottomLeftX, bottomLeftY, bottomRightX, bottomRightY
        0.0, CAMERA_IMAGE_HEIGHT - 1, CAMERA_IMAGE_WIDTH - 1, CAMERA_IMAGE_HEIGHT - 1);
    public static TrcHomographyMapper.Rectangle worldRect;

}   //class RobotParams
