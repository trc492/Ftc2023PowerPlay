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

import java.util.concurrent.atomic.AtomicBoolean;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcSensor;
import TrcCommonLib.trclib.TrcThresholdTrigger;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcDigitalInput;
import TrcFtcLib.ftclib.FtcDistanceSensor;
import TrcFtcLib.ftclib.FtcMotorActuator;

/**
 * This class encapsulates a PID Actuator as the Turret. The turret has additional capabilities over a PID actuator.
 * It contains a REV 2m distance sensor that will detect the proximity of the target (e.g. pole or cone).
 */
public class Turret
{
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    private final TrcDbgTrace msgTracer;
    public final FtcDcMotor motor;
    private final TrcPidActuator pidTurret;
    private final FtcDigitalInput calDirectionSwitch;
    private final FtcDistanceSensor sensor;
    private final TrcThresholdTrigger thresholdTrigger;
    private double prevTurretPower = 0.0;
    private String currOwner = null;
    public double rawMotorPosition;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     * @param tracePidInfo specifies true to enable PidInfo tracing, false to disable, only valid if msgTracer not null.
     */
    public Turret(String instanceName, TrcDbgTrace msgTracer, boolean tracePidInfo)
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

        this.msgTracer = msgTracer;
        FtcMotorActuator motorActuator = new FtcMotorActuator(instanceName, motorParams, turretParams);
        motor = motorActuator.getMotor();
        pidTurret = motorActuator.getPidActuator();
        if (msgTracer != null)
        {
            pidTurret.setMsgTracer(msgTracer, tracePidInfo);
        }

        calDirectionSwitch = new FtcDigitalInput(
            instanceName + ".dirSwitch", RobotParams.TURRET_DIR_SWITCH_INVERTED);
        if (RobotParams.Preferences.hasTurretSensor)
        {
            sensor = new FtcDistanceSensor(instanceName + ".sensor");
            thresholdTrigger = new TrcThresholdTrigger(
                instanceName + ".thresholdTrigger", this::getSensorValue, this::thresholdTriggerEvent);
            thresholdTrigger.setTrigger(
                RobotParams.TURRET_SENSOR_LOWER_THRESHOLD, RobotParams.TURRET_SENSOR_UPPER_THRESHOLD,
                RobotParams.TURRET_SENSOR_SETTLING_PERIOD);
        }
        else
        {
            sensor = null;
            thresholdTrigger = null;
        }
    }   //Turret

    /**
     * This method acquires exclusive ownership of the subsystem if it's not already owned by somebody else.
     *
     * @param owner specifies the ID string of the caller requesting ownership.
     * @return true if successfully acquired ownership, false otherwise.
     */
    public boolean acquireExclusiveAccess(String owner)
    {
        final String funcName = "acquireExclusiveAccess";
        boolean success = pidTurret.acquireExclusiveAccess(owner);

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "owner=%s, success=%s", owner, success);
        }

        if (success)
        {
            currOwner = owner;
        }

        return success;
    }   //acquireExclusiveAccess

    /**
     * This method release exclusive ownership of the subsystem if the caller is indeed the owner.
     *
     * @param owner specifies the ID string of the caller releasing ownership.
     * @return true if successfully releasing ownership, false otherwise.
     */
    public boolean releaseExclusiveAccess(String owner)
    {
        final String funcName = "releaseExclusiveAccess";
        boolean success = pidTurret.releaseExclusiveAccess(owner);

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "owner=%s, success=%s", owner, success);
        }

        if (success)
        {
            currOwner = null;
        }

        return success;
    }   //releaseExclusiveAccess

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
        double value = 0.0;

        if (sensor != null)
        {
            TrcSensor.SensorData<Double> data = sensor.getProcessedData(0, FtcDistanceSensor.DataType.DISTANCE_INCH);
            value = data.value;
        }

        return value;
    }   //getSensorValue

    /**
     * This method enables/disables the threshold trigger.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setTriggerEnabled(boolean enabled)
    {
        final String funcName = "setTriggerEnabled";

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "enabled=%s", enabled);
        }

        thresholdTrigger.setEnabled(enabled);
    }   //setTriggerEnabled

    /**
     * This method returns the data recorded during the trigger settling period.
     *
     * @return trigger settling data.
     */
    public Double[] getTriggerSettlingData()
    {
        return thresholdTrigger.getTriggerSettlingData();
    }   //getTriggerSettlingData

    /**
     * This method checks if the target is detected.
     *
     * @return true if target is detected, false otherwise.
     */
    public boolean detectedTarget()
    {
        final String funcName = "detectedTarget";
        double value = getSensorValue();
        boolean detected = value >= RobotParams.TURRET_SENSOR_LOWER_THRESHOLD &&
                           value <= RobotParams.TURRET_SENSOR_UPPER_THRESHOLD;

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "sensorValue=%.2f, detected=%s", value, detected);
        }

        return detected;
    }   //detectedTarget

    /**
     * This method returns the current turret position in degrees.
     *
     * @return turret position in degrees.
     */
    public double getPosition()
    {
        rawMotorPosition = motor.motor.getCurrentPosition() * RobotParams.TURRET_DEG_PER_COUNT;
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
        pidTurret.zeroCalibrate(owner, RobotParams.TURRET_CAL_POWER, null);
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
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     */
    public void cancel(String owner)
    {
        if (owner == null || owner.equals(currOwner))
        {
            pidTurret.cancel(owner);
            thresholdTrigger.setEnabled(false);
        }
    }   //cancel

    /**
     * This method stops the turret if it was in motion.
     */
    public void cancel()
    {
        cancel(currOwner);
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
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param target specifies the target position of the turret in degrees.
     * @param holdTarget specifies true to hold target, false otherwise.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param timeout specifies timeout in seconds for the operation.
     */
    public void setTarget(
        String owner, double target, boolean holdTarget, double powerLimit, TrcEvent event,
        double timeout)
    {
        pidTurret.setTarget(owner, 0.0, target, holdTarget, powerLimit, event, timeout);
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
     * This method is called when the sensor value falls in the threshold range for at least the settling period.
     *
     * @param context specifies the callback context.
     */
    private void thresholdTriggerEvent(Object context)
    {
        final String funcName = "sensorTriggerEvent";
        boolean isActive = ((AtomicBoolean) context).get();

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "value=%.3f, state=%s", getSensorValue(), isActive);
        }

        if (isActive)
        {
            cancel(currOwner);
        }
    }   //thresholdTriggerEvent

}   //class Turret
