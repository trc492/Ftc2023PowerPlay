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
        public static boolean useAprilTag = true;
        public static boolean showAprilTagView = true;
        public static boolean useTraceLog = true;
        public static boolean useBatteryMonitor = false;
        public static boolean useLoopPerformanceMonitor = true;
        public static boolean useVelocityControl = false;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "Robot3543";
    public static final String TEAM_FOLDER_PATH                 =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
    public static final String LOG_FOLDER_PATH                  = TEAM_FOLDER_PATH + "/logs";
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
    //
    // Robot dimensions.
    //
    public static final double ROBOT_LENGTH                     = 17.0;
    public static final double ROBOT_WIDTH                      = 13.0;
    public static final double DRIVE_BASE_LENGTH                = 13.25;
    public static final double DRIVE_BASE_WIDTH                 = 11.25;
    //
    // Game positions.
    //
    public static final double STARTPOS_FROM_FIELDCENTER_X      = 1.5 * FULL_TILE_INCHES;
    public static final double STARTPOS_FROM_FIELDCENTER_Y      = HALF_FIELD_INCHES - ROBOT_LENGTH/2.0;
    public static TrcPose2D STARTPOS_RED_LEFT = new TrcPose2D(
        -STARTPOS_FROM_FIELDCENTER_X, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    public static TrcPose2D STARTPOS_RED_RIGHT = new TrcPose2D(
        STARTPOS_FROM_FIELDCENTER_X, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    public static TrcPose2D STARTPOS_BLUE_LEFT = new TrcPose2D(
        STARTPOS_FROM_FIELDCENTER_X, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
    public static TrcPose2D STARTPOS_BLUE_RIGHT = new TrcPose2D(
        -STARTPOS_FROM_FIELDCENTER_X, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
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
    public static final double STEER_LOW_LIMIT                  = -90.0;
    public static final double STEER_HIGH_LIMIT                 = 90.0;
    public static final double LFSTEER_MINUS90                  = 0.20;
    public static final double LFSTEER_PLUS90                   = 0.85;
    public static final double RFSTEER_MINUS90                  = 0.21;
    public static final double RFSTEER_PLUS90                   = 0.87;
    public static final double LBSTEER_MINUS90                  = 0.26;
    public static final double LBSTEER_PLUS90                   = 0.94;
    public static final double RBSTEER_MINUS90                  = 0.18;
    public static final double RBSTEER_PLUS90                   = 0.85;

    public static final boolean LFDRIVE_INVERTED                = true;
    public static final boolean RFDRIVE_INVERTED                = true;
    public static final boolean LBDRIVE_INVERTED                = true;
    public static final boolean RBDRIVE_INVERTED                = true;
    public static final boolean LFSTEER_INVERTED                = false;
    public static final boolean RFSTEER_INVERTED                = false;
    public static final boolean LBSTEER_INVERTED                = false;
    public static final boolean RBSTEER_INVERTED                = false;

    public static final DcMotor.RunMode DRIVE_MOTOR_MODE        = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final boolean DRIVE_WHEEL_BRAKE_MODE_ON       = true;
    public static final boolean LEFT_WHEEL_INVERTED             = true;
    public static final boolean RIGHT_WHEEL_INVERTED            = false;
    public static final double TURN_POWER_LIMIT                 = 0.5;
    public static final double SLOW_DRIVE_POWER_SCALE           = 0.5;
    public static final double X_ODOMETRY_WHEEL_OFFSET          = ROBOT_LENGTH/2.0 - (3.875 + 9.5); //behind centroid
    public static final double Y_LEFT_ODOMETRY_WHEEL_OFFSET     = -15.25/2.0;
    public static final double Y_RIGHT_ODOMETRY_WHEEL_OFFSET    = 15.25/2.0;
    public static final RobotDrive.DriveMode ROBOT_DRIVE_MODE   = RobotDrive.DriveMode.HOLONOMIC_MODE;
    //
    // Velocity controlled constants.
    //
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS     = GOBILDA_5203_312_MAX_VELOCITY_PPS;

    public static TrcPidController.PidCoefficients xPosPidCoeff =
        new TrcPidController.PidCoefficients(0.095, 0.0, 0.001, 0.0);
    public static double XPOS_TOLERANCE                         = 1.0;
    public static double XPOS_INCHES_PER_COUNT                  = 0.01924724265461924299065420560748;

    public static TrcPidController.PidCoefficients yPosPidCoeff =
        new TrcPidController.PidCoefficients(0.06, 0.0, 0.002, 0.0);
    public static final double YPOS_TOLERANCE                   = 1.0;
    public static final double YPOS_INCHES_PER_COUNT            = 0.02166184604662450653409090909091;

    public static TrcPidController.PidCoefficients turnPidCoeff =
        new TrcPidController.PidCoefficients(0.025, 0.0, 0.001, 0.0);
    public static final double TURN_TOLERANCE                   = 2.0;

    public static final double X_ODWHEEL_INCHES_PER_COUNT       = 7.6150160901199168116026724971383e-4;
    public static final double Y_ODWHEEL_INCHES_PER_COUNT       = 7.6301149255006038191364659148717e-4;
    //
    // Pure Pursuit parameters.
    //
    // No-Load max velocity (i.e. theoretical maximum)
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    public static final double ROBOT_MAX_VELOCITY               = 25.0;     // measured maximum from drive speed test.
    public static final double ROBOT_MAX_ACCELERATION           = 3380.0;   // measured maximum from drive speed test.
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static TrcPidController.PidCoefficients velPidCoeff  =
        new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/ROBOT_MAX_VELOCITY);
    public static final double PPD_FOLLOWING_DISTANCE           = 6.0;
    public static final double PPD_POS_TOLERANCE                = 2.0;
    public static final double PPD_TURN_TOLERANCE               = 1.0;
    //
    // Vision subsystem.
    //
    public static final double CAMERA_FRONT_OFFSET              = 7.5;  //Camera offset from front of robot in inches
    public static final double CAMERA_LEFT_OFFSET               = 6.0;  //Camera offset from left of robot in inches
    public static final double CAMERA_HEIGHT_OFFSET             = 16.0; //Camera offset from floor in inches
    public static final double CAMERA_TILT_DOWN                 = 36.0; //Camera tilt down angle from horizontal in deg
    public static final double CAMERA_TAGSIZE                   = 0.035;// in meters
    // Camera: Logitech C310
    public static final int CAMERA_IMAGE_WIDTH                  = 640;
    public static final int CAMERA_IMAGE_HEIGHT                 = 480;
    public static final double CAMERA_FX                        = 821.993;  // in pixels
    public static final double CAMERA_FY                        = 821.993;  // in pixels
    public static final double CAMERA_CX                        = 330.489;  // in pixels
    public static final double CAMERA_CY                        = 248.997;  // in pixels
    public static final int FRAME_QUEUE_CAPACITY                = 2;

    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X      = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y      = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X     = CAMERA_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y     = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X   = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y   = CAMERA_IMAGE_HEIGHT - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X  = CAMERA_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y  = CAMERA_IMAGE_HEIGHT - 1;

    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // Measurement unit: inches
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X       = -22.25;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y       = 60.0;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X      = 23.0;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y      = 60.0;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X    = -8.75;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y    = 16.0;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X   = 7.5;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y   = 16.0;

    public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
    public static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);

}   //class RobotParams
