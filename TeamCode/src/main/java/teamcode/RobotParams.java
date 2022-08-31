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

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;

import TrcCommonLib.trclib.TrcHomographyMapper;

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
        static boolean noRobot = false;
        static boolean initSubsystems = true;
        static boolean useExternalOdometry = false;
        static boolean useBlinkin = true;
        static boolean useVuforia = false;
        static boolean showVuforiaView = false;
        static boolean useTensorFlow = false;
        static boolean showTensorFlowView = false;
        static boolean useEasyOpenCV = false;
        static boolean showEasyOpenCvView = false;
        static boolean useTraceLog = true;
        static boolean useBatteryMonitor = false;
        static boolean useLoopPerformanceMonitor = true;
        static boolean useVelocityControl = false;
    }   //class Preferences

    public enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
        ARCADE_MODE
    }   //enum DriveMode

    static final String LOG_PATH_FOLDER                         =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftclogs";
    //
    // Hardware names.
    //
    static final String HWNAME_IMU                              = "imu";
    static final String HWNAME_WEBCAM                           = "Webcam 1";
    static final String HWNAME_BLINKIN                          = "blinkin";
    static final String HWNAME_LFDRIVE_MOTOR                    = "lfDriveMotor";
    static final String HWNAME_RFDRIVE_MOTOR                    = "rfDriveMotor";
    static final String HWNAME_LBDRIVE_MOTOR                    = "lbDriveMotor";
    static final String HWNAME_RBDRIVE_MOTOR                    = "rbDriveMotor";
    static final String HWNAME_LFSTEER_SERVO1                   = "lfSteerServo1";
    static final String HWNAME_LFSTEER_SERVO2                   = "lfSteerServo2";
    static final String HWNAME_RFSTEER_SERVO1                   = "rfSteerServo1";
    static final String HWNAME_RFSTEER_SERVO2                   = "rfSteerServo2";
    static final String HWNAME_LBSTEER_SERVO1                   = "lbSteerServo1";
    static final String HWNAME_LBSTEER_SERVO2                   = "lbSteerServo2";
    static final String HWNAME_RBSTEER_SERVO1                   = "rbSteerServo1";
    static final String HWNAME_RBSTEER_SERVO2                   = "rbSteerServo2";
    //
    // Field dimensions.
    //
    static final double FULL_FIELD_INCHES                       = 141.0;
    static final double HALF_FIELD_INCHES                       = FULL_FIELD_INCHES/2.0;
    static final double FULL_TILE_INCHES                        = 23.75;
    static final double HALF_TILE_INCHES                        = FULL_TILE_INCHES/2.0;
    //
    // Robot dimensions.
    //
    static final double ROBOT_LENGTH                            = 17.0;
    static final double ROBOT_WIDTH                             = 13.0;
    static final double ROBOT_DRIVEBASE_WIDTH                   = 17.0;     // Required by swerve drive base.
    static final double ROBOT_DRIVEBASE_LENGTH                  = 17.0;     // Required by swerve drive base.
    //
    // Game positions.
    //

    //
    // Motor Odometries.
    //
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double GOBILDA_5203_312_ENCODER_PPR            = ((((1.0 + (46.0/17.0)))*(1.0 + (46.0/11.0)))*28.0);
    static final double GOBILDA_5203_312_RPM                    = 312.0;
    static final double GOBILDA_5203_312_MAX_VELOCITY_PPS       =
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // 2795.987 pps
    //
    // DriveBase subsystem.
    //
    static final DriveMode ROBOT_DRIVE_MODE                     = DriveMode.ARCADE_MODE;
    static final DcMotor.RunMode DRIVE_MOTOR_MODE               = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final boolean LEFT_WHEEL_INVERTED                    = true;
    static final boolean RIGHT_WHEEL_INVERTED                   = false;
    static final boolean DRIVE_WHEEL_BRAKE_MODE                 = true;
    static final double TURN_POWER_LIMIT                        = 0.5;
    static final double SLOW_DRIVE_POWER_SCALE                  = 0.5;
    static final double X_ODOMETRY_WHEEL_OFFSET                 = ROBOT_LENGTH/2.0 - (3.875 + 9.5); //behind centroid
    static final double Y_LEFT_ODOMETRY_WHEEL_OFFSET            = -15.25/2.0;
    static final double Y_RIGHT_ODOMETRY_WHEEL_OFFSET           = 15.25/2.0;
    //
    // Velocity controlled constants.
    //
    static final double DRIVE_MOTOR_MAX_VELOCITY_PPS            = GOBILDA_5203_312_MAX_VELOCITY_PPS;

    static final double MECANUM_X_KP                            = 0.095;
    static final double MECANUM_X_KI                            = 0.0;
    static final double MECANUM_X_KD                            = 0.0;
    static final double MECANUM_X_KF                            = 0.0;
    static final double MECANUM_X_TOLERANCE                     = 1.0;
    static final double MECANUM_X_INCHES_PER_COUNT              = 0.01924724265461924299065420560748;

    static final double MECANUM_Y_KP                            = 0.04;
    static final double MECANUM_Y_KI                            = 0.0;
    static final double MECANUM_Y_KD                            = 0.005;
    static final double MECANUM_Y_KF                            = 0.0;
    static final double MECANUM_Y_TOLERANCE                     = 1.0;
    static final double MECANUM_Y_INCHES_PER_COUNT              = 0.02166184604662450653409090909091;

    public static final double SWERVE_KP                        = 0.02;
    public static final double SWERVE_KI                        = 0.0;
    public static final double SWERVE_KD                        = 0.0;
    public static final double SWERVE_KF                        = 0.0;
    public static final double SWERVE_TOLERANCE                 = 2.0;
    public static final double SWERVE_INCHES_PER_COUNT          = 9.072106867127145344367826764411e-4;

    static final double GYRO_TURN_KP                            = 0.013;
    static final double GYRO_TURN_KI                            = 0.0;
    static final double GYRO_TURN_KD                            = 0.0;
    static final double GYRO_TURN_KF                            = 0.0;
    static final double GYRO_TURN_TOLERANCE                     = 1.0;

    static final double ODWHEEL_X_INCHES_PER_COUNT              = 7.6150160901199168116026724971383e-4;
    static final double ODWHEEL_Y_INCHES_PER_COUNT              = 7.6301149255006038191364659148717e-4;
    //
    // Pure Pursuit parameters.
    //
    // No-Load max velocity (i.e. theoretical maximum)
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    static final double ROBOT_MAX_VELOCITY                      = 25.0;     // measured maximum from drive speed test.
    static final double ROBOT_MAX_ACCELERATION                  = 3380.0;   // measured maximum from drive speed test.
    static final double ROBOT_VEL_KP                            = 0.0;
    static final double ROBOT_VEL_KI                            = 0.0;
    static final double ROBOT_VEL_KD                            = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    static final double ROBOT_VEL_KF                            = 1.0 / ROBOT_MAX_VELOCITY;
    static final double PPD_FOLLOWING_DISTANCE                  = 6.0;
    static final double PPD_POS_TOLERANCE                       = 2.0;
    static final double PPD_TURN_TOLERANCE                      = 1.0;
    //
    // Vision subsystem.
    //
    static final String TRACKABLE_IMAGES_FILE                   = "FreightFrenzy";
    static final double CAMERA_FRONT_OFFSET                     = 7.5;  //Camera offset from front of robot in inches
    static final double CAMERA_HEIGHT_OFFSET                    = 16.0; //Camera offset from floor in inches
    static final double CAMERA_LEFT_OFFSET                      = 6.0;  //Camera offset from left of robot in inches
    static final int CAMERA_IMAGE_WIDTH                         = 320;
    static final int CAMERA_IMAGE_HEIGHT                        = 240;
    static final int FRAME_QUEUE_CAPACITY                       = 2;

    static final double HOMOGRAPHY_CAMERA_TOPLEFT_X             = 0.0;
    static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y             = 0.0;
    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X            = CAMERA_IMAGE_WIDTH - 1;
    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y            = 0.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X          = 0.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y          = CAMERA_IMAGE_HEIGHT - 1;
    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X         = CAMERA_IMAGE_WIDTH - 1;
    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y         = CAMERA_IMAGE_HEIGHT - 1;

    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // Measurement unit: inches
    static final double HOMOGRAPHY_WORLD_TOPLEFT_X              = -22.25;
    static final double HOMOGRAPHY_WORLD_TOPLEFT_Y              = 60.0;
    static final double HOMOGRAPHY_WORLD_TOPRIGHT_X             = 23.0;
    static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y             = 60.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X           = -8.75;
    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y           = 16.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X          = 7.5;
    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y          = 16.0;

    static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
    static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);

}   //class RobotParams
