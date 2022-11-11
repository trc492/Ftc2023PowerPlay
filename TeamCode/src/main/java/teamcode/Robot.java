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

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Scanner;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcIntake;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcFtcLib.ftclib.FtcAndroidTone;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRevBlinkin;
import TrcFtcLib.ftclib.FtcRobotBattery;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    //
    // Global objects.
    //
    public FtcOpMode opMode;
    public FtcDashboard dashboard;
    public TrcDbgTrace globalTracer;
    public static FtcMatchInfo matchInfo = null;
    //
    // Vision subsystems.
    //
    public Vision vision;
    //
    // Sensors and indicators.
    //
    public FtcRevBlinkin blinkin;
    public FtcRobotBattery battery;
    public FtcAndroidTone androidTone;
    //
    // Subsystems.
    //
    public RobotDrive robotDrive;
    public TrcPidActuator elevator = null;
    public TrcPidActuator arm = null;
    public Turret turret;
    public TrcIntake intake = null;
    public TaskCyclingCones cyclingTask;

    //zero intake

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *                specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTextView(
            ((FtcRobotControllerActivity)opMode.hardwareMap.appContext)
                .findViewById(com.qualcomm.ftcrobotcontroller.R.id.textOpMode));
        globalTracer = TrcDbgTrace.getGlobalTracer();

        speak("Init starting");
        //
        // Initialize vision subsystems.
        //
        if ((RobotParams.Preferences.useVuforia ||
             RobotParams.Preferences.useTensorFlow ||
             RobotParams.Preferences.useEasyOpenCV) &&
            (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            vision = new Vision(this);
        }
        //
        // If noRobot is true, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (!RobotParams.Preferences.noRobot)
        {
            //
            // Create and initialize sensors and indicators.
            //
            if (RobotParams.Preferences.useBlinkin)
            {
                blinkin = new FtcRevBlinkin(RobotParams.HWNAME_BLINKIN);
                //
                // Vision uses Blinkin as an indicator, so set it up.
                //
                if (vision != null)
                {
                    vision.setupBlinkin();
                }
            }

            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }

            androidTone = new FtcAndroidTone("androidTone");
            //
            // Create and initialize RobotDrive.
            //
            robotDrive = RobotParams.Preferences.swerveRobot? new SwerveDrive(this): new MecanumDrive(this);
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.initSubsystems)
            {
                if (RobotParams.Preferences.useElevator)
                {
                    final FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams(
                        RobotParams.ELEVATOR_MOTOR_INVERTED,
                        RobotParams.ELEVATOR_HAS_LOWER_LIMIT_SWITCH, RobotParams.ELEVATOR_LOWER_LIMIT_INVERTED,
                        RobotParams.ELEVATOR_HAS_UPPER_LIMIT_SWITCH, RobotParams.ELEVATOR_UPPER_LIMIT_INVERTED);
                    final TrcPidActuator.Parameters elevatorParams = new TrcPidActuator.Parameters()
                        .setPosRange(RobotParams.ELEVATOR_MIN_POS, RobotParams.ELEVATOR_MAX_POS)
                        .setScaleOffset(RobotParams.ELEVATOR_INCHES_PER_COUNT, RobotParams.ELEVATOR_OFFSET)
                        .setPidParams(new TrcPidController.PidParameters(
                            RobotParams.ELEVATOR_KP, RobotParams.ELEVATOR_KI, RobotParams.ELEVATOR_KD,
                            RobotParams.ELEVATOR_TOLERANCE))
                        .setPowerCompensation(this::getElevatorPowerCompensation)
//                        .setStallProtectionParams(
//                            RobotParams.ELEVATOR_STALL_MIN_POWER, RobotParams.ELEVATOR_STALL_TOLERANCE,
//                            RobotParams.ELEVATOR_STALL_TIMEOUT, RobotParams.ELEVATOR_RESET_TIMEOUT)
                        .setZeroCalibratePower(RobotParams.ELEVATOR_CAL_POWER)
                        .setPresetTolerance(RobotParams.ELEVATOR_PRESET_TOLERANCE)
                        .setPosPresets(RobotParams.ELEVATOR_PRESET_LEVELS);
                    elevator = new FtcMotorActuator(
                        RobotParams.HWNAME_ELEVATOR, motorParams, elevatorParams).getPidActuator();
                    elevator.getPidController().setOutputRange(-RobotParams.ELEVATOR_DOWN_POWER_SCALE, 1.0);
                    elevator.setMsgTracer(globalTracer);
                    elevator.setBeep(androidTone);
                }

                if (RobotParams.Preferences.useArm)
                {
                    final FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams(
                        RobotParams.ARM_MOTOR_INVERTED,
                        RobotParams.ARM_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_LOWER_LIMIT_INVERTED,
                        RobotParams.ARM_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_UPPER_LIMIT_INVERTED);
                    final TrcPidActuator.Parameters armParams = new TrcPidActuator.Parameters()
                        .setPosRange(RobotParams.ARM_MIN_POS, RobotParams.ARM_MAX_POS)
                        .setScaleOffset(RobotParams.ARM_DEG_PER_COUNT, RobotParams.ARM_OFFSET)
                        .setPidParams(new TrcPidController.PidParameters(
                            RobotParams.ARM_KP, RobotParams.ARM_KI, RobotParams.ARM_KD, RobotParams.ARM_TOLERANCE))
//                        .setStallProtectionParams(
//                            RobotParams.ARM_STALL_MIN_POWER, RobotParams.ARM_STALL_TOLERANCE,
//                            RobotParams.ARM_STALL_TIMEOUT, RobotParams.ARM_RESET_TIMEOUT)
                        .setZeroCalibratePower(RobotParams.ARM_CAL_POWER)
                        .setPresetTolerance(RobotParams.ARM_PRESET_TOLERANCE)
                        .setPosPresets(RobotParams.ARM_PRESET_LEVELS);
                    arm = new FtcMotorActuator(RobotParams.HWNAME_ARM, motorParams, armParams).getPidActuator();
                    arm.setMsgTracer(globalTracer);
                    arm.setBeep(androidTone);
                }

                if(RobotParams.Preferences.useTurret)
                {
                    turret = new Turret(this);
//                    turret.zeroCalibrate();
                }

                if (RobotParams.Preferences.useIntake)
                {
                    TrcIntake.Parameters intakeParams = new TrcIntake.Parameters()
                        .setMotorInverted(true)
                        .setTriggerInverted(true)
                        .setAnalogThreshold(RobotParams.INTAKE_SENSOR_THRESHOLD)
                        .setMsgTracer(globalTracer);
                    intake = new Intake(RobotParams.HWNAME_INTAKE, intakeParams).getTrcIntake();
                }
                //
                // Create and initialize auto-assist tasks.
                //
                cyclingTask = new TaskCyclingCones(this);
                this.cyclingTask.setMsgTracer(globalTracer);
            }
        }

        speak("Init complete");
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return RobotParams.ROBOT_NAME;
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We need odometry in TeleOp for GridDrive.
            //
            robotDrive.driveBase.setOdometryEnabled(true);
        }
        //
        // The following are performance counters, could be disabled for competition if you want.
        // But it might give you some insight if somehow autonomous wasn't performing as expected.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.setElapsedTimerEnabled(true);
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "stopMode";
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Stop all auto-assists tasks if any.
        //
        cyclingTask.cancel();
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.vuforiaVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling Vuforia.");
                vision.vuforiaVision.setEnabled(false);
            }

            if (vision.tensorFlowVision != null)
            {
                globalTracer.traceInfo(funcName, "Shutting down TensorFlow.");
                vision.tensorFlowShutdown();
            }

            if (vision.frontEocvVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling FrontEocvVision.");
                vision.frontEocvVision.setEnabled(false);
            }

            if (vision.elevatorEocvVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling ElevatorEocvVision.");
                vision.elevatorEocvVision.setEnabled(false);
            }
        }

        if (robotDrive != null)
        {
            //
            // Disable odometry.
            //
            robotDrive.driveBase.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    /**
     * This method returns the power required to make the elevator gravity neutral.
     *
     * @param currPower specifies the current motor power.
     * @return elevator gravity compensation power.
     */
    private double getElevatorPowerCompensation(double currPower)
    {
        return Math.abs(elevator.getPosition() - RobotParams.ELEVATOR_MIN_POS) <= RobotParams.ELEVATOR_TOLERANCE?
                0.0: RobotParams.ELEVATOR_POWER_COMPENSATION;
    }   //getElevatorPowerCompensation

    /**
     * This method saves the current robot position to a data file. Typically, this is called at the end of autonomous
     * so that at the beginning of TeleOp, we can restore the robot pose from this file.
     */
    public void saveCurrentRobotPose()
    {
        final String funcName = "saveCurrentRobotPose";

        try (PrintStream out = new PrintStream(new FileOutputStream(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.CURRENT_ROBOT_POSE_FILE)))
        {
            TrcPose2D currPose = robotDrive.driveBase.getFieldPosition();

            out.println(currPose.x + "," + currPose.y + "," + currPose.angle);
            out.close();
            globalTracer.traceInfo(funcName, "Saved robot pose=%s", currPose);
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveCurrentRobotPose

    /**
     * This method is typically called at the beginning of TeleOp to read the Robot Pose data file saved by the
     * previous autonomous session. It restore the current robot's field position to this robot pose so that TeleOp
     * can continue with the same position at the end of autonomous.
     *
     * @return true if the data is read successful, false if failed such as file not found.
     */
    public boolean restoreCurrentRobotPose()
    {
        final String funcName = "restoreCurrentRobotPose";
        boolean success = true;

        try (Scanner in = new Scanner(new FileReader(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.CURRENT_ROBOT_POSE_FILE)))
        {
            String[] poseData = in.nextLine().split(",", 3);
            if (poseData.length == 3)
            {
                TrcPose2D robotPose = new TrcPose2D(
                    Double.parseDouble(poseData[0]), Double.parseDouble(poseData[1]), Double.parseDouble(poseData[2]));
                robotDrive.driveBase.setFieldPosition(robotPose);
                globalTracer.traceInfo(funcName, "Restore RobotPose=%s", robotPose);
            }
            else
            {
                success = false;
                globalTracer.traceWarn(funcName, "Invalid data (len=%d).", poseData.length);
            }
        }
        catch (FileNotFoundException e)
        {
            success = false;
            globalTracer.traceWarn(funcName, "Current Robot Pose data file not found.");
        }

        return success;
    }   //restoreCurrentRobotPose

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
