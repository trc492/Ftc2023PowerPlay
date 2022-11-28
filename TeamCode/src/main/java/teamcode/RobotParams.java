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
import TrcFtcLib.ftclib.FtcGamepad;

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
        // Miscellaneous
        public static boolean useTraceLog = true;
        public static boolean useLoopPerformanceMonitor = true;
        public static boolean useBlinkin = true;
        public static boolean useBatteryMonitor = false;
        // Vision
        public static boolean useVuforia = false;
        public static boolean showVuforiaView = false;
        public static boolean useTensorFlow = false;
        public static boolean showTensorFlowView = false;
        public static boolean useEasyOpenCV = true;
        public static boolean showEasyOpenCvView = true;
        // Robot
        public static boolean noRobot = false;
        public static boolean swerveRobot = false;
        // Drive Base
        public static boolean useExternalOdometry = true;
        public static boolean useVelocityControl = false;
        // Subsystems
        public static boolean initSubsystems = true;
        public static boolean useTurret = true;
        public static boolean useElevator = true;
        public static boolean useArm = true;
        public static boolean useGrabber = true;
        public static boolean hasGrabberSensor = true;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "Robot3543";
    public static final String TEAM_FOLDER_PATH                 =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
    public static final String LOG_FOLDER_PATH                  = TEAM_FOLDER_PATH + "/logs";
    public static final String STEERING_CALIBRATION_DATA_FILE   = "SteerCalibration.txt";
    //
    // Hardware names.
    //
    public static final String HWNAME_IMU                       = "imu";
    public static final String HWNAME_FRONT_WEBCAM              = "Webcam 1";
    public static final String HWNAME_ELEVATOR_WEBCAM           = "Webcam 2";
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
    public static final String HWNAME_TURRET                    = "turret";
    public static final String HWNAME_TURRET_DIR_SWITCH         = "turretDirSwitch";
    public static final String HWNAME_ELEVATOR                  = "elevator";
    public static final String HWNAME_ARM                       = "arm";
    public static final String HWNAME_GRABBER                   = "grabber";
    //
    // Field dimensions.
    //
    public static final double FULL_FIELD_INCHES                = 141.0;
    public static final double HALF_FIELD_INCHES                = FULL_FIELD_INCHES/2.0;
    public static final double FULL_TILE_INCHES                 = 23.75;
    //
    // Robot dimensions.
    //
    public static final double ROBOT_LENGTH                     = 17.375;
    public static final double ROBOT_WIDTH                      = 14.625;
    public static final double DRIVE_BASE_LENGTH                = 13.250;
    public static final double DRIVE_BASE_WIDTH                 = 12.375;
    //
    // Game positions.
    //
    public static final double STARTPOS_FROM_FIELDCENTER_X      = 1.5 * FULL_TILE_INCHES;
    public static final double STARTPOS_FROM_FIELDCENTER_Y      = HALF_FIELD_INCHES - ROBOT_LENGTH/2.0;

    // Robot start positions in inches.
    public static final TrcPose2D STARTPOS_RED_LEFT = new TrcPose2D(
        -STARTPOS_FROM_FIELDCENTER_X, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D STARTPOS_RED_RIGHT = new TrcPose2D(
        STARTPOS_FROM_FIELDCENTER_X, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D STARTPOS_BLUE_LEFT = new TrcPose2D(
        STARTPOS_FROM_FIELDCENTER_X, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_RIGHT = new TrcPose2D(
        -STARTPOS_FROM_FIELDCENTER_X, STARTPOS_FROM_FIELDCENTER_Y, 180.0);

    // Robot park positions in tile units.
    public static final double PARKPOS_LEFTP1_FROM_FIELDCENTER_X= 2.5;
    public static final double PARKPOS_LEFTP2_FROM_FIELDCENTER_X= 1.5;
    public static final double PARKPOS_LEFTP3_FROM_FIELDCENTER_X= 0.5;
    public static final double PARKPOS_FAR_FROM_FIELDCENTER_Y   = 0.5;
    public static final double PARKPOS_NEAR_FROM_FIELDCENTER_Y  = 1.5;

    public static final TrcPose2D PARKPOS_RED_LEFTP1_FAR = new TrcPose2D(
        -PARKPOS_LEFTP1_FROM_FIELDCENTER_X, -PARKPOS_FAR_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D PARKPOS_RED_LEFTP1_NEAR = new TrcPose2D(
        -PARKPOS_LEFTP1_FROM_FIELDCENTER_X, -PARKPOS_NEAR_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D PARKPOS_RED_LEFTP2_FAR = new TrcPose2D(
        -PARKPOS_LEFTP2_FROM_FIELDCENTER_X, -PARKPOS_FAR_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D PARKPOS_RED_LEFTP2_NEAR = new TrcPose2D(
        -PARKPOS_LEFTP2_FROM_FIELDCENTER_X, -PARKPOS_NEAR_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D PARKPOS_RED_LEFTP3_FAR = new TrcPose2D(
        -PARKPOS_LEFTP3_FROM_FIELDCENTER_X, -PARKPOS_FAR_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D PARKPOS_RED_LEFTP3_NEAR = new TrcPose2D(
        -PARKPOS_LEFTP3_FROM_FIELDCENTER_X, -PARKPOS_NEAR_FROM_FIELDCENTER_Y, 0.0);

    public static final TrcPose2D[] PARKPOS_RED_LEFT_NEAR = {
        PARKPOS_RED_LEFTP1_NEAR,
        PARKPOS_RED_LEFTP2_NEAR,
        PARKPOS_RED_LEFTP3_NEAR
    };
    public static final TrcPose2D[] PARKPOS_RED_LEFT_FAR = {
        PARKPOS_RED_LEFTP1_FAR,
        PARKPOS_RED_LEFTP2_FAR,
        PARKPOS_RED_LEFTP3_FAR
    };

    // Auto-Assist navigation locations in tile units.
    public static final TrcPose2D AUTONAV_CONESTACK_RED_LEFT    = new TrcPose2D(-1.5, -0.5, -90.0);
    public static final TrcPose2D AUTONAV_CONESTACK_RED_RIGHT   = new TrcPose2D(1.5, -0.5, 90.0);
    public static final TrcPose2D AUTONAV_CONESTACK_BLUE_LEFT   = new TrcPose2D(1.5, 0.5, 90.0);
    public static final TrcPose2D AUTONAV_CONESTACK_BLUE_RIGHT  = new TrcPose2D(-1.5, 0.5, -90.0);

    public static final TrcPose2D AUTONAV_SUBSTATION_RED_LEFT   = new TrcPose2D(-0.5, -2.2, 180.0);
    public static final TrcPose2D AUTONAV_SUBSTATION_RED_RIGHT  = new TrcPose2D(0.5, -2.2, 180.0);
    public static final TrcPose2D AUTONAV_SUBSTATION_BLUE_LEFT  = new TrcPose2D(0.5, 2.2, 0.0);
    public static final TrcPose2D AUTONAV_SUBSTATION_BLUE_RIGHT = new TrcPose2D(-0.5, 2.2, 0.0);

    public static final TrcPose2D HIGHPOLE_SUBSTATION_RED       = new TrcPose2D (0.0, -1.0, 0.0);
    public static final TrcPose2D HIGHPOLE_SUBSTATION_BLUE      = new TrcPose2D (0.0, 1.0, 0.0);
    public static final TrcPose2D HIGHPOLE_AUDIENCE             = new TrcPose2D (-1.0, 0.0, 0.0);
    public static final TrcPose2D HIGHPOLE_REFEREE              = new TrcPose2D (1.0, 0.0, 0.0);

    public static final TrcPose2D[] AUTONAV_PICKUP_RED = {
        AUTONAV_SUBSTATION_RED_LEFT,
        AUTONAV_SUBSTATION_RED_RIGHT,
        AUTONAV_CONESTACK_RED_LEFT,
        AUTONAV_CONESTACK_RED_RIGHT
    };
    public static final TrcPose2D[] AUTONAV_PICKUP_BLUE = {
        AUTONAV_SUBSTATION_BLUE_LEFT,
        AUTONAV_SUBSTATION_BLUE_RIGHT,
        AUTONAV_CONESTACK_BLUE_LEFT,
        AUTONAV_CONESTACK_BLUE_RIGHT
    };
    public static final int AUTONAV_LEFT_SUBSTATION_INDEX       = 0;
    public static final int AUTONAV_RIGHT_SUBSTATION_INDEX      = 1;
    public static final int AUTONAV_LEFT_CONESTACK_INDEX        = 2;
    public static final int AUTONAV_RIGHT_CONESTACK_INDEX       = 3;

    public static final TrcPose2D[] AUTONAV_HIGHPOLES = {
        HIGHPOLE_SUBSTATION_RED,
        HIGHPOLE_SUBSTATION_BLUE,
        HIGHPOLE_AUDIENCE,
        HIGHPOLE_REFEREE
    };

    // In tile units.
    public static final TrcPose2D CONE_STACK_RED_LEFT           = new TrcPose2D(-2.3, -0.5, -90.0);
    public static final TrcPose2D LOOK_FOR_CONE_POS_LEFT        = new TrcPose2D(-2.0, -0.5, -90.0);

    public static final TrcPose2D SCORE_LOCATION_RED_LEFT       = new TrcPose2D(-0.5, -1.0, 180.0);
    public static final TrcPose2D SUBSTATION_RED_LEFT           = new TrcPose2D(-0.55, -1.95, 150.0);
    // Game element dimensions
    public static final double STACKED_CONE_ADDITION            = 1.5;
    public static final double LOW_JUNCTION_HEIGHT              = 20.0;
    public static final double MID_JUNCTION_HEIGHT              = 26.5;
    public static final double HIGH_JUNCTION_HEIGHT             = 36.0;
    //
    // Vision subsystem.
    //
    public static final int FRAME_QUEUE_CAPACITY                = 2;
    public static final double APRILTAG_SIZE                    = 0.043;// in meters
    public static final double APRILTAG_HEIGHT_OFFSET           = 1.5;  // in inches

    public static final double FRONTCAM_FRONT_OFFSET            = 2.000;//Camera offset from front of robot in inches
    public static final double FRONTCAM_LEFT_OFFSET             = 7.125;//Camera offset from left of robot in inches
    public static final double FRONTCAM_HEIGHT_OFFSET           = 3.750;//Camera offset from floor in inches
    public static final double FRONTCAM_TILT_DOWN               = 15.00;//Camera tilt down angle from horizontal in deg
    // Front Camera: Logitech C310
    public static final int FRONTCAM_IMAGE_WIDTH                = 640;
    public static final int FRONTCAM_IMAGE_HEIGHT               = 480;
    public static final double FRONTCAM_FX                      = 821.993;  // in pixels
    public static final double FRONTCAM_FY                      = 821.993;  // in pixels
    public static final double FRONTCAM_CX                      = 330.489;  // in pixels
    public static final double FRONTCAM_CY                      = 248.997;  // in pixels
    // Elevator Camera: Logitech C310
    public static final int ELEVATORCAM_IMAGE_WIDTH             = 640;
    public static final int ELEVATORCAM_IMAGE_HEIGHT            = 480;
    public static final double ELEVATORCAM_VERTICAL_FOV         = 26.0;     // in degrees
    public static final double ELEVATORCAM_ANGLE_PER_PIXEL      = ELEVATORCAM_VERTICAL_FOV/ELEVATORCAM_IMAGE_HEIGHT;

    // Measurement unit: pixels
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X      = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y      = 120.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X     = FRONTCAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y     = 120.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X   = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y   = FRONTCAM_IMAGE_HEIGHT - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X  = FRONTCAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y  = FRONTCAM_IMAGE_HEIGHT - 1;

    // Measurement unit: inches
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X       = -12.5625;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y       = 48.0 - ROBOT_LENGTH + FRONTCAM_FRONT_OFFSET;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X      = 11.4375;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y      = 44.75 - ROBOT_LENGTH + FRONTCAM_FRONT_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X    = -2.5625;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y    = 21.0 - ROBOT_LENGTH + FRONTCAM_FRONT_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X   = 2.5626;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y   = 21.0 - ROBOT_LENGTH + FRONTCAM_FRONT_OFFSET;

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
    //
    // Motor Odometries.
    //
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_312_ENCODER_PPR     = (((1.0 + 46.0/17.0)*(1.0 + 46.0/11.0))*28.0);
    public static final double GOBILDA_5203_312_RPM             = 312.0;
    public static final double GOBILDA_5203_312_MAX_VELOCITY_PPS=
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // 2795.9872 pps
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_435_ENCODER_PPR     = (((1.0 + 46.0/17.0)*(1.0 + 46.0/17.0))*28.0);
    public static final double GOBILDA_5203_435_RPM             = 435.0;
    public static final double GOBILDA_5203_435_MAX_VELOCITY_PPS=
        GOBILDA_5203_435_ENCODER_PPR*GOBILDA_5203_435_RPM/60.0; // 2787.9135 pps
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
    public static final FtcGamepad.DriveMode ROBOT_DRIVE_MODE   = FtcGamepad.DriveMode.ARCADE_MODE;
    //
    // Velocity controlled constants.
    //
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS     = GOBILDA_5203_312_MAX_VELOCITY_PPS;

    public static final TrcPidController.PidCoefficients xPosPidCoeff =
        new TrcPidController.PidCoefficients(0.095, 0.0, 0.001, 0.0);
    public static final double XPOS_TOLERANCE                   = 1.0;
    public static final double XPOS_INCHES_PER_COUNT            = 0.01924724265461924299065420560748;
    public static final Double X_RAMP_RATE                      = null;//10.0;

    public static final TrcPidController.PidCoefficients yPosPidCoeff =
        new TrcPidController.PidCoefficients(0.06, 0.0, 0.002, 0.0);
    public static final double YPOS_TOLERANCE                   = 1.0;
    public static final double YPOS_INCHES_PER_COUNT            = 0.02166184604662450653409090909091;
    public static final Double Y_RAMP_RATE                      = null;//10.0;

    public static final TrcPidController.PidCoefficients turnPidCoeff =
        new TrcPidController.PidCoefficients(0.025, 0.02, 0.000, 0.0, 30.0);
    public static final double TURN_SETTLING                    = TrcPidController.DEF_SETTLING_TIME;
    public static final double TURN_TOLERANCE                   = 1.0;
    public static final double TURN_STEADY_STATE_ERR            = 2.0;
    public static final double TURN_STALL_ERRRATE_THRESHOLD     = 1.0;
    public static final Double TURN_RAMP_RATE                   = null;//10.0;

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
    public static final double ROBOT_MAX_VELOCITY               = 23.0;     // measured maximum from drive speed test.
    public static final double ROBOT_MAX_ACCELERATION           = 3000.0;   // measured maximum from drive speed test.
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static TrcPidController.PidCoefficients velPidCoeff  =
        new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/ROBOT_MAX_VELOCITY);
    public static final double PPD_FOLLOWING_DISTANCE           = 6.0;
    public static final double PPD_POS_TOLERANCE                = 2.0;
    public static final double PPD_TURN_TOLERANCE               = 1.0;
    //
    // Grid Drive parameters
    //
    public static final double GRIDDRIVE_CELL_SIZE              = FULL_TILE_INCHES;
    //
    // Elevator Subsystem
    //
    static final double ELEVATOR_KP                             = 0.5;
    static final double ELEVATOR_KI                             = 0.0;
    static final double ELEVATOR_KD                             = 0.025;
    static final double ELEVATOR_TOLERANCE                      = 2.0;
    static final double ELEVATOR_ENCODER_PPR                    = GOBILDA_5203_435_ENCODER_PPR;
    static final double ELEVATOR_PULLEY_DIAMETER                = 1.5*1.06;         // in inches
    static final double ELEVATOR_PULLEY_CIRCUMFERENCE           = Math.PI*ELEVATOR_PULLEY_DIAMETER;
    static final double ELEVATOR_INCHES_PER_COUNT               = ELEVATOR_PULLEY_CIRCUMFERENCE/ELEVATOR_ENCODER_PPR;
    static final double ELEVATOR_OFFSET                         = 7.0;              // in inches
    static final double ELEVATOR_MIN_POS                        = 7.0;
    static final double ELEVATOR_MAX_POS                        = 36.0;
    static final double ELEVATOR_MIN_POS_FOR_TURRET             = 10.0;
    static final double ELEVATOR_POS_FOR_TURRET_TURN            = ELEVATOR_MIN_POS_FOR_TURRET + 2.0;
    static final boolean ELEVATOR_MOTOR_INVERTED                = false;
    static final boolean ELEVATOR_HAS_LOWER_LIMIT_SWITCH        = true;
    static final boolean ELEVATOR_LOWER_LIMIT_INVERTED          = false;
    static final boolean ELEVATOR_HAS_UPPER_LIMIT_SWITCH        = false;
    static final boolean ELEVATOR_UPPER_LIMIT_INVERTED          = false;
    static final double ELEVATOR_CAL_POWER                      = -0.1;
    static final double ELEVATOR_POWER_COMPENSATION             = 0.1;
    static final double ELEVATOR_STALL_MIN_POWER                = 0.3;
    static final double ELEVATOR_STALL_TOLERANCE                = 0.0;
    static final double ELEVATOR_STALL_TIMEOUT                  = 1.0;
    static final double ELEVATOR_RESET_TIMEOUT                  = 0.5;
    static final double ELEVATOR_CONE_GRAB_HEIGHT               = 3;//18.0;
    static final double ELEVATOR_SCORING_HEIGHT                 = 33.0;
    static final double ELEVATOR_PRESET_TOLERANCE               = 2.0;
    public static final double HIGH_JUNCTION_SCORING_HEIGHT     = 32.0;
    public static final double CAPPING_OFFSET                   = -8.0;
    static final double[] ELEVATOR_PRESET_LEVELS                = new double[] {
        ELEVATOR_MIN_POS,                                       // Slot 0
        ELEVATOR_CONE_GRAB_HEIGHT,                              // Slot 1
//        STACKED_CONE_ADDITION + ELEVATOR_CONE_GRAB_HEIGHT,      // Slot 2
//        STACKED_CONE_ADDITION * 2 + ELEVATOR_CONE_GRAB_HEIGHT,  // Slot 3
//        STACKED_CONE_ADDITION * 3 + ELEVATOR_CONE_GRAB_HEIGHT,  // Slot 4
//        STACKED_CONE_ADDITION * 4 + ELEVATOR_CONE_GRAB_HEIGHT,  // Slot 5
//        LOW_JUNCTION_HEIGHT,                                  // Slot 6
//        MID_JUNCTION_HEIGHT,                                  // Slot 7
        HIGH_JUNCTION_HEIGHT,                                   // Slot 8
        ELEVATOR_MAX_POS                                        // Slot 9
    };
    // 0.0 for placeholder so 1 additional cone per index
    static final double[] ELEVATOR_PICKUP_PRESETS               = new double[] {
        0.0,
        ELEVATOR_CONE_GRAB_HEIGHT,                              // Slot 1
        STACKED_CONE_ADDITION + ELEVATOR_CONE_GRAB_HEIGHT,      // Slot 2
        STACKED_CONE_ADDITION * 2 + ELEVATOR_CONE_GRAB_HEIGHT,  // Slot 3
        STACKED_CONE_ADDITION * 3 + ELEVATOR_CONE_GRAB_HEIGHT,  // Slot 4
        STACKED_CONE_ADDITION * 4 + ELEVATOR_CONE_GRAB_HEIGHT,  // Slot 5
    };
    static final double ELEVATOR_DOWN_POWER_SCALE               = 0.2;
    //
    // Arm subsystem.
    //
    static final double ARM_KP                                  = 0.15;
    static final double ARM_KI                                  = 0.0;
    static final double ARM_KD                                  = 0.0;
    static final double ARM_TOLERANCE                           = 0.5;
    static final double ARM_ENCODER_PPR                         = GOBILDA_5203_435_ENCODER_PPR;
    // https://www.gobilda.com/super-duty-worm-drive-pan-kit-28-1-ratio/
    static final double ARM_GEAR_RATIO                          = 28.0;
    static final double ARM_DEG_PER_COUNT                       = (360.0/(ARM_ENCODER_PPR*ARM_GEAR_RATIO));
    static final double ARM_OFFSET                              = 7.0;
    /*s
     * Arm Zero-Calibrated to Up Position
     * Arm Max Position is Down
     */
    static final double ARM_MIN_POS                             = 8.0;
    static final double ARM_MAX_POS                             = 140.0;

    static final double ARM_MIN_POS_FOR_TURRET                  = 65.0;
    static final double ARM_POS_FOR_TURRET_TURN                 = ARM_MIN_POS_FOR_TURRET - 5.0;
    static final boolean ARM_MOTOR_INVERTED                     = true;
    static final boolean ARM_HAS_LOWER_LIMIT_SWITCH             = true;
    static final boolean ARM_LOWER_LIMIT_INVERTED               = false;
    static final boolean ARM_HAS_UPPER_LIMIT_SWITCH             = false;
    static final boolean ARM_UPPER_LIMIT_INVERTED               = false;
    static final double ARM_CAL_POWER                           = -0.3;
    static final double ARM_STALL_MIN_POWER                     = 0.3;
    static final double ARM_STALL_TOLERANCE                     = 0.0;
    static final double ARM_STALL_TIMEOUT                       = 1.0;
    static final double ARM_RESET_TIMEOUT                       = 0.5;
    static final double ARM_RETRACTED                           = ARM_MIN_POS + 3;
    static final double ARM_SCORE_POS                           = 20;
    static final double ARM_SINGLE_CONE_POS                     = ARM_MAX_POS;
    static final double ARM_STACK_2_POS                         = ARM_MAX_POS;
    static final double ARM_STACK_3_CONES_POS                   = ARM_MAX_POS;
    static final double ARM_STACK_4_CONES_POS                   = ARM_MAX_POS;
    static final double ARM_STACK_5_CONES_POS                   = ARM_MAX_POS;
    static final double ARM_HORIZONTAL                          = 90.0;
    static final double ARM_PRESET_TOLERANCE                    = 2.0;
    static final double[] ARM_PRESET_LEVELS                     = new double[] {
        ARM_MIN_POS,                                            // Slot 0: Up position
        ARM_SCORE_POS,                                          // Slot 1
        ARM_HORIZONTAL,                                         // Slot 2
        ARM_MAX_POS                                             // Slot 2: Down position
    };
    //0 is a placeholder so index 1 is 1 cone, 2 is 2 cones, etc.
    static final double ARM_PICKUP_POS                          = 100;
    static final double[] ARM_PICKUP_PRESETS                    = new double[] {
        0.0,
        ARM_SINGLE_CONE_POS,
        ARM_STACK_2_POS,
        ARM_STACK_3_CONES_POS,
        ARM_STACK_4_CONES_POS,
        ARM_STACK_5_CONES_POS
    };
    //
    // Turret subsystem.
    //
    static final double TURRET_KP                               = 0.06;
    static final double TURRET_KI                               = 0.0;
    static final double TURRET_KD                               = 0.0;
    static final double TURRET_TOLERANCE                        = 1.0;
    // Rev Core Hex Motor (https://www.revrobotics.com/rev-41-1300/)
    static final double TURRET_ENCODER_PPR                      = 288.0;
    static final double TURRET_GEAR_RATIO                       = 100.0/60.0;
    static final double TURRET_DEG_PER_COUNT                    = 360.0/(TURRET_ENCODER_PPR*TURRET_GEAR_RATIO);
    static final double TURRET_OFFSET                           = 0.0;
    static final double TURRET_MIN_POS                          = 0.0;
    static final double TURRET_MAX_POS                          = 359.0;
    // continuously?
    static final boolean TURRET_MOTOR_INVERTED                  = true;
    static final boolean TURRET_HAS_LOWER_LIMIT_SWITCH          = true;
    static final boolean TURRET_LOWER_LIMIT_INVERTED            = true;
    static final boolean TURRET_HAS_UPPER_LIMIT_SWITCH          = false;
    static final boolean TURRET_UPPER_LIMIT_INVERTED            = false;
    static final boolean TURRET_DIR_SWITCH_INVERTED             = false;
    static final double TURRET_CAL_POWER                        = 0.3;
    static final double TURRET_POWER_SCALE                      = 1.0;
    static final double TURRET_STALL_MIN_POWER                  = 0.3;
    static final double TURRET_STALL_TOLERANCE                  = 0.0;
    static final double TURRET_STALL_TIMEOUT                    = 1.0;
    static final double TURRET_RESET_TIMEOUT                    = 0.5;
    static final double TURRET_BACK                             = 0.0;
    static final double TURRET_RIGHT                            = 90.0;
    static final double TURRET_FRONT                            = 180.0;
    static final double TURRET_LEFT                             = 270.0;
    static final double TURRET_PRESET_TOLERANCE                 = 10.0;
    static final double[] TURRET_PRESET_LEVELS                  = new double[] {
        TURRET_BACK,    // Slot 0
        TURRET_RIGHT,   // Slot 1
        TURRET_FRONT,   // Slot 2
        TURRET_LEFT     // Slot 3
    };
    //In tile coordinates---how far from the cone the robot should be before trying to pick it up
    static final double TURRET_PICKUP_OFFSET                    = 0.3; //todo: tune this
    //
    // Grabber subsystem.
    //
    static final double GRABBER_MAX_STEPRATE                    = 1.0;
    static final double GRABBER_MIN_POS                         = 0.0;
    static final double GRABBER_MAX_POS                         = 0.25;
    static final boolean GRABBER_LSERVO_INVERTED                = false;
    static final boolean GRABBER_RSERVO_INVERTED                = true;
    static final boolean GRABBER_TRIGGER_INVERTED               = true;
    static final double GRABBER_SENSOR_THRESHOLD                = 3.0;
    static final double GRABBER_OPEN_POS                        = GRABBER_MAX_POS;
    static final double GRABBER_OPEN_TIME                       = 0.5;
    static final double GRABBER_CLOSE_POS                       = GRABBER_MIN_POS;
    static final double GRABBER_CLOSE_TIME                      = 0.5;

}   //class RobotParams
