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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcIntake;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcFtcLib.ftclib.FtcAndroidTone;
import TrcFtcLib.ftclib.FtcDashboard;
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
    public TaskTileGridDrive tileGridDriveTask;

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
                        .setStallProtectionParams(
                            RobotParams.ARM_STALL_MIN_POWER, RobotParams.ARM_STALL_TOLERANCE,
                            RobotParams.ARM_STALL_TIMEOUT, RobotParams.ARM_RESET_TIMEOUT)
                        .setZeroCalibratePower(RobotParams.ARM_CAL_POWER)
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
                tileGridDriveTask = new TaskTileGridDrive(this);
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
            // Enable odometry only for autonomous or test modes.
            //
            if (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE)
            {
                robotDrive.driveBase.setOdometryEnabled(true);
            }
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
        double compensationPower =
            Math.abs(elevator.getPosition() - RobotParams.ELEVATOR_MIN_POS) <= RobotParams.ELEVATOR_TOLERANCE?
                0.0: RobotParams.ELEVATOR_POWER_COMPENSATION;

        return compensationPower;
    }   //getElevatorPowerCompensation

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
