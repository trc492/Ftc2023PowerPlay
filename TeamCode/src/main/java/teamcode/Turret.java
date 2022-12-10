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

import TrcCommonLib.trclib.TrcAnalogSensorTrigger;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcFtcLib.ftclib.FtcDigitalInput;
import TrcFtcLib.ftclib.FtcDistanceSensor;
import TrcFtcLib.ftclib.FtcMotorActuator;

/**
 * This class encapsulates a PID Actuator as the Turret. The turret has additional capabilities over a PID actuator.
 * It contains a REV 2m distance sensor that will detect the proximity of the pole. It provides an auto-assist method
 * to allow the turret to turn and find the pole. Once it's found it calculates the distance from the grabber to the
 * pole so that it will move the grabber towards the pole for scoring the cone.
 */
public class Turret
{
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    private static final String moduleName = "Turret";
    private final TrcPidActuator pidTurret;
    private final FtcDigitalInput calDirectionSwitch;
    private final TrcAnalogSensorTrigger<FtcDistanceSensor.DataType> analogTrigger;
    private double prevTurretPower = 0.0;

    public Turret(TrcDbgTrace msgTracer, boolean tracePidInfo)
    {
        final FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams(
            RobotParams.TURRET_MOTOR_INVERTED,
            RobotParams.TURRET_HAS_LOWER_LIMIT_SWITCH, RobotParams.TURRET_LOWER_LIMIT_INVERTED,
            RobotParams.TURRET_HAS_UPPER_LIMIT_SWITCH, RobotParams.TURRET_UPPER_LIMIT_INVERTED, true);
        final TrcPidActuator.Parameters turretParams = new TrcPidActuator.Parameters()
            .setPosRange(RobotParams.TURRET_MIN_POS, RobotParams.TURRET_MAX_POS)
            .setScaleOffset(RobotParams.TURRET_DEG_PER_COUNT, RobotParams.TURRET_OFFSET)
            .resetPositionOnLowerLimit(false)
            .setPidParams(new TrcPidController.PidParameters(
                RobotParams.TURRET_KP, RobotParams.TURRET_KI, RobotParams.TURRET_KD,
                RobotParams.TURRET_TOLERANCE))
            .setPresetTolerance(RobotParams.TURRET_PRESET_TOLERANCE)
            .setPosPresets(RobotParams.TURRET_PRESET_LEVELS);

        pidTurret = new FtcMotorActuator(RobotParams.HWNAME_TURRET, motorParams, turretParams).getPidActuator();
        if (msgTracer != null)
        {
            pidTurret.setMsgTracer(msgTracer, tracePidInfo);
        }

        calDirectionSwitch = new FtcDigitalInput(
            RobotParams.HWNAME_TURRET + ".dirSwitch", RobotParams.TURRET_DIR_SWITCH_INVERTED);
        if (RobotParams.Preferences.hasTurretSensor)
        {
            FtcDistanceSensor sensor = new FtcDistanceSensor(RobotParams.HWNAME_TURRET + ".poleSensor");
            analogTrigger = new TrcAnalogSensorTrigger<FtcDistanceSensor.DataType>(
                RobotParams.HWNAME_TURRET + ".analogTrigger", sensor, 0, FtcDistanceSensor.DataType.DISTANCE_INCH,
                new double[]{RobotParams.TURRET_SENSOR_THRESHOLD}, false, this::analogTriggerEvent);
        }
        else
        {
            analogTrigger = null;
        }
    }   //Turret

    /**
     * This method returns the PID actuator object.
     *
     * @return PID Actuator.
     */
    public TrcPidActuator getPidActuator()
    {
        return pidTurret;
    }   //getPidActuator

    /**
     * This method reads the distance sensor value.
     *
     * @return distance in inches.
     */
    public double getSensorValue()
    {
        return analogTrigger != null? analogTrigger.getSensorValue(): 0.0;
    }   //getSensorValue

    /**
     * This method checks if the pole is detected.
     *
     * @return true if pole is detected, false otherwise.
     */
    public boolean detectedPole()
    {
        return analogTrigger != null && analogTrigger.getSensorValue() <= RobotParams.TURRET_SENSOR_THRESHOLD;
    }   //detectedPole

    /**
     * This method returns the current turret position in degrees.
     *
     * @return turret position in degrees.
     */
    public double getPosition()
    {
        return pidTurret.getPosition();
    }   //getPosition

    /**
     * This method checks if the zero position switch is active.
     *
     * @return true if the zero position switch is active, false otherwise.
     */
    public boolean isZeroPosSwitchActive()
    {
        return pidTurret.isLowerLimitSwitchActive();
    }   //isZeroPosSwitchActive

    /**
     * This method checks if the zero calibration direction switch is active.
     *
     * @return true if the zero calibration direction switch is active, false otherwise.
     */
    public boolean isCalDirSwitchActive()
    {
        return calDirectionSwitch.isActive();
    }   //isCalDirSwitchActive

    /**
     * This method zero calibrates the turret
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     */
    public void zeroCalibrate(String owner)
    {
        double calPower = Math.abs(RobotParams.TURRET_CAL_POWER);
        pidTurret.zeroCalibrate(owner, calDirectionSwitch.isActive()? calPower: -calPower, null);
    }   //zeroCalibrate

    /**
     * This method zero calibrates the turret
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    }   //zeroCalibrate

    /**
     * This method stops the turret if it was in motion.
     */
    public void cancel()
    {
        pidTurret.cancel();
        analogTrigger.setEnabled(false);
    }   //cancel

    /**
     * This method sets the turret target position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before performing the action.
     * @param target specifies the target position of the turret in degrees.
     * @param holdTarget specifies true to hold target, false otherwise.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param timeout specifies timeout in seconds for the operation.
     */
    public void setTarget(
        String owner, double delay, double target, boolean holdTarget, double powerLimit, TrcEvent event,
        double timeout)
    {
        pidTurret.setTarget(owner, delay, target, holdTarget, powerLimit, event, timeout);
    }   //setTarget

    /**
     * This method sets the turret target position.
     *
     * @param delay specifies the delay in seconds before performing the action.
     * @param target specifies the target position of the turret in degrees.
     * @param holdTarget specifies true to hold target, false otherwise.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param timeout specifies timeout in seconds for the operation.
     */
    public void setTarget(
        double delay, double target, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
    {
        pidTurret.setTarget(null, delay, target, holdTarget, powerLimit, event, timeout);
    }   //setTarget

    /**
     * This method sets the turret target position.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param holdTarget specifies true to hold target, false otherwise.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param timeout specifies timeout in seconds for the operation.
     */
    public void setTarget(double target, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
    {
        pidTurret.setTarget(null, 0.0, target, holdTarget, powerLimit, event, timeout);
    }   //setTarget

    /**
     * This method sets the turret target position.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param holdTarget specifies true to hold target, false otherwise.
     * @param powerLimit specifies the maximum power the turret will turn.
     */
    public void setTarget(double target, boolean holdTarget, double powerLimit)
    {
        pidTurret.setTarget(null, 0.0, target, holdTarget, powerLimit, null, 0.0);
    }   //setTarget

    /**
     * This method sets the turret target position.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param holdTarget specifies true to hold target, false otherwise.
     */
    public void setTarget(double target, boolean holdTarget)
    {
        pidTurret.setTarget(null, 0.0, target, holdTarget, 1.0, null, 0.0);
    }   //setTarget

    /**
     * This method sets the turret to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     */
    public void setPresetPosition(String owner, int preset)
    {
        if (pidTurret.validatePresetIndex(preset))
        {
            setTarget(owner, 0.0, RobotParams.TURRET_PRESET_LEVELS[preset], true, 1.0, null, 0.0);
        }
    }   //setPresetPosition

    /**
     * This method sets the turret to the specified preset position.
     */
    public void setPresetPosition(int preset)
    {
        if (pidTurret.validatePresetIndex(preset))
        {
            setTarget(null, 0.0, RobotParams.TURRET_PRESET_LEVELS[preset], true, 1.0, null, 0.0);
        }
    }   //setPresetPosition

    /**
     * This method moves the turret to the next position up the preset list.
     */
    public void presetPositionUp()
    {
        setPresetPosition(pidTurret.nextPresetIndexUp());
    }   //presetPositionUp

    /**
     * This method moves the turret to the next position down the preset list.
     */
    public void presetPositionDown()
    {
        setPresetPosition(pidTurret.nextPresetIndexDown());
    }   //presetPositionDown

    /**
     * This method sets the turret in motion with the given power but it will first check if it's safe to turn the
     * turret. If not, it will instead raise the arm to above the safe level. Since setPower is generally called by
     * TeleOp code, it will not do the actual setPower after raising the arm because by the time the arm is raised
     * the gamepad control may have a different turret power value. Therefore, the TeleOp code will call again with
     * the new power value and this time it is safe to turn the turret.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param power specifies the power value to turn the turret.
     * @param usePid specifies true to use PID control, false otherwise.
     */
    public void setPower(String owner, double power, boolean usePid)
    {
        if (power != prevTurretPower)
        {
            if (power == 0.0)
            {
                cancel();
                pidTurret.setPidPower(owner, power, false);
            }
            else if (usePid)
            {
                pidTurret.setPidPower(owner, power, false);
            }
            else
            {
                pidTurret.setPower(owner, power);
            }
            prevTurretPower = power;
        }
    }   //setPower

    /**
     * This method sets the turret in motion with the given power but it will first check if it's safe to turn the
     * turret. If not, it will instead raise the arm to above the safe level. Since setPower is generally called by
     * TeleOp code, it will not do the actual setPower after raising the arm because by the time the arm is raised
     * the gamepad control may have a different turret power value. Therefore, the TeleOp code will call again with
     * the new power value and this time it is safe to turn the turret.
     *
     * @param power specifies the power value to turn the turret.
     * @param usePid specifies true to use PID control, false otherwise.
     */
    public void setPower(double power, boolean usePid)
    {
        setPower(null, power, usePid);
    }   //setPower

    /**
     * This method sets the turret in motion with the given power but it will first check if it's safe to turn the
     * turret. If not, it will instead raise the arm to above the safe level. Since setPower is generally called by
     * TeleOp code, it will not do the actual setPower after raising the arm because by the time the arm is raised
     * the gamepad control may have a different turret power value. Therefore, the TeleOp code will call again with
     * the new power value and this time it is safe to turn the turret.
     *
     * @param power specifies the power value to turn the turret.
     */
    public void setPower(double power)
    {
        setPower(null, power, true);
    }   //setPower

    /**
     * This method turns the turret in an attempt to find the pole. It turns the turret in the direction of the
     * specified relative target. The turret will stop either it has reached the target or the pole is found. If
     * the pole is found, it will terminate the turret turn.
     *Preconditions: arm must be above a certain limit, elevator must be above a certain value
     *
     * @param relativeTarget specifies the relative target in degrees (negative to turn left, positive to turn right).
     * @param powerLimit specifies how fast the turret should turn.
     * @param event specifies the event to signal when done, can be null if none provided.
     * @param timeout specifies the maximum amount of time for the completion of this operation in seconds.
     */
    public void autoAssistFindPole(double relativeTarget, double powerLimit, TrcEvent event, double timeout)
    {
        analogTrigger.setEnabled(true);
        setTarget(getPosition() + relativeTarget, true, powerLimit, event, timeout);
    }   //autoAssistFindPole
    public void enableTurretAutoAssist(){

        analogTrigger.setEnabled(true);
    }
    public double scoringArmAngle(double sensorDistance) {
        return 90.0 + RobotParams.ARM_ANGLE_OFFSET - Math.toDegrees(Math.acos((sensorDistance + RobotParams.CLAW_DISTANCE_ADUSTMENT)/RobotParams.ARM_JOINT_LENGTH));
    }
    public double calculateArmAngle(){
        double sensorDistance = analogTrigger.getSensorValue();
        return scoringArmAngle(sensorDistance);
    }

    /**
     * This method is called when an analog sensor threshold has been crossed.
     *
     * @param context specifies the callback context.
     */
    private void analogTriggerEvent(Object context)
    {
        final String funcName = "analogTriggerEvent";
        TrcAnalogSensorTrigger.CallbackContext callbackContext = (TrcAnalogSensorTrigger.CallbackContext) context;

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "Zone=%d->%d, value=%.3f",
                callbackContext.prevZone, callbackContext.currZone, callbackContext.sensorValue);
        }

        if (detectedPole())
        {
            cancel();
        }
    }   //analogTriggerEvent


}   //class Turret
