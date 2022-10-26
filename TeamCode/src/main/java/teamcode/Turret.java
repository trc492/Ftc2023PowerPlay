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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcNotifier;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcFtcLib.ftclib.FtcDigitalInput;
import TrcFtcLib.ftclib.FtcMotorActuator;

/**
 * This class encapsulates a PID Actuator as the Turret. Since the turret is not safe to swing around unless the
 * intake is raised to a safe height so that it won't hit the drive base motors and electronics, we will provide
 * the overriding methods of setPower and setTarget that will make sure the intake is raised above "safe level"
 * before turning the turret.
 */
public class Turret
{
    private final Robot robot;
    private final FtcDigitalInput calDirectionSwitch;
    private final TrcPidActuator pidTurret;
    private double turretTarget = 0.0;
    private double turretPowerLimit = 1.0;
    private TrcEvent turretEvent = null;
    private TrcNotifier.Receiver turretCallback = null;
    private boolean armLevelSafe = false;
    private boolean elevatorLevelSafe = false;

    public Turret(Robot robot)
    {
        this.robot = robot;
        calDirectionSwitch = new FtcDigitalInput(
            RobotParams.HWNAME_TURRET_DIR_SWITCH, RobotParams.TURRET_DIR_SWITCH_INVERTED);

        final FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams(
            RobotParams.TURRET_MOTOR_INVERTED,
            RobotParams.TURRET_HAS_LOWER_LIMIT_SWITCH, RobotParams.TURRET_LOWER_LIMIT_INVERTED,
            RobotParams.TURRET_HAS_UPPER_LIMIT_SWITCH, RobotParams.TURRET_UPPER_LIMIT_INVERTED);
        final TrcPidActuator.Parameters turretParams = new TrcPidActuator.Parameters()
            .setPosRange(RobotParams.TURRET_MIN_POS, RobotParams.TURRET_MAX_POS)
            .setScaleOffset(RobotParams.TURRET_DEG_PER_COUNT, RobotParams.TURRET_OFFSET)
            .setPidParams(new TrcPidController.PidParameters(
                RobotParams.TURRET_KP, RobotParams.TURRET_KI, RobotParams.TURRET_KD,
                RobotParams.TURRET_TOLERANCE))
            .setStallProtectionParams(
                RobotParams.TURRET_STALL_MIN_POWER, RobotParams.TURRET_STALL_TOLERANCE,
                RobotParams.TURRET_STALL_TIMEOUT, RobotParams.TURRET_RESET_TIMEOUT)
            .setZeroCalibratePower(RobotParams.TURRET_CAL_POWER)
            .setPosPresets(RobotParams.TURRET_PRESET_LEVELS);
        pidTurret = new FtcMotorActuator(
            RobotParams.HWNAME_TURRET, motorParams, turretParams).getPidActuator();
    }   //Turret

    /**
     * This method zero calibrates the turret by first zero calibrating the arm and elevator. Once the arm and
     * elevator are zero calibrated, we will know the exact arm and elevator positions. Also, the arm is zero
     * calibrated upward. This allows the turret to turn without hitting anything. Therefore, zero calibrating
     * the turret involves 3 steps:
     *  1. zero calibrate the elevator.
     *  2. zero calibrate the arm and wait for its calibration to be done (chain to the armZeroCalDone handler).
     *  3. In the armZeroCalDone handler, do a fire and forget zero calibrate on the turret.
     */
    public void zeroCalibrate()
    {
        robot.elevator.zeroCalibrate(RobotParams.ELEVATOR_CAL_POWER);
        robot.arm.zeroCalibrate(RobotParams.ARM_CAL_POWER, this::armZeroCalDoneCallback);
    }   //zeroCalibrate

    /**
     * This method is called after the arm zero calibration is done so that we know it's safe to zero calibrate the
     * turret.
     *
     * @param context not used.
     */
    private void armZeroCalDoneCallback(Object context)
    {
        pidTurret.zeroCalibrate(
            calDirectionSwitch.isActive()? RobotParams.TURRET_CAL_POWER: -RobotParams.TURRET_CAL_POWER);
    }   //armZeroCalDoneCallback

    /**
     * This method sets the turret target position. It first checks if it's safe for the turret to turn without
     * hitting anything. If it's not safe, it will first raise the arm above the "safe level" before setting the
     * target for the turret.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param callback specifies the notify callback to call when the turret is on target, can be null if not provided.
     */
    public void setTarget(double target, double powerLimit, TrcEvent event, TrcNotifier.Receiver callback)
    {
        double armPos = robot.arm.getPosition();
        double elevatorPos = robot.elevator.getPosition();

        armLevelSafe = armPos <= RobotParams.ARM_MIN_POS_FOR_TURRET;
        elevatorLevelSafe = elevatorPos >= RobotParams.ELEVATOR_MIN_POS_FOR_TURRET;
        if (!turnTurretToPos(target, powerLimit, event, callback))
        {
            // Either the arm or the elevator are not at safe level, we need to raise them first before the turn.
            turretTarget = target;
            turretPowerLimit = powerLimit;
            turretEvent = event;
            turretCallback = callback;

            if (!armLevelSafe)
            {
                robot.arm.setTarget(
                    RobotParams.ARM_MIN_POS_FOR_TURRET, false, 1.0, null, this::armRaiseDoneCallback);
            }

            if (!elevatorLevelSafe)
            {
                robot.elevator.setTarget(
                    RobotParams.ELEVATOR_MIN_POS_FOR_TURRET, true, 1.0, null, this::elevatorRaiseDoneCallback);
            }
        }
    }   //setTarget

    /**
     * This method sets the turret target position. It first checks if it's safe for the turret to turn without
     * hitting anything. If it's not safe, it will first raise the arm above the "safe level" before setting the
     * target for the turret.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param powerLimit specifies the maximum power the turret will turn.
     */
    public void setTarget(double target, double powerLimit)
    {
        setTarget(target, powerLimit, null, null);
    }   //setTarget

    /**
     * This method sets the turret target position. It first checks if it's safe for the turret to turn without
     * hitting anything. If it's not safe, it will first raise the arm above the "safe level" before setting the
     * target for the turret.
     *
     * @param target specifies the target position of the turret in degrees.
     */
    public void setTarget(double target)
    {
        setTarget(target, 1.0, null, null);
    }   //setTarget

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
        double armPos = robot.arm.getPosition();
        double elevatorPos = robot.elevator.getPosition();

        armLevelSafe = armPos <= RobotParams.ARM_MIN_POS_FOR_TURRET;
        elevatorLevelSafe = elevatorPos >= RobotParams.ELEVATOR_MIN_POS_FOR_TURRET;
        if (!turnTurretWithPower(power))
        {
            if (!armLevelSafe)
            {
                robot.arm.setTarget(RobotParams.ARM_MIN_POS_FOR_TURRET, false);
            }

            if (!elevatorLevelSafe)
            {
                robot.elevator.setTarget(RobotParams.ELEVATOR_MIN_POS_FOR_TURRET, true);
            }
        }
    }   //setPower

    /**
     * This method checks if the arm and elevator positions are safe to turn the turret to the given position without
     * hitting anything.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param callback specifies the notify callback to call when the turret is on target, can be null if not provided.
     * @return true if it was safe and we successfully initiated the turn, false if we did not turn.
     */
    private boolean turnTurretToPos(double target, double powerLimit, TrcEvent event, TrcNotifier.Receiver callback)
    {
        boolean doTurn = armLevelSafe && elevatorLevelSafe;

        if (doTurn)
        {
            pidTurret.setTarget(target, true, powerLimit, event, callback);
        }

        return doTurn;
    }   //turnTurretToPos

    /**
     * This method checks if the arm and elevator positions are safe to turn the turret at the given power without
     * hitting anything.
     *
     * @param power specifies the power to turn the turret.
     * @return true if it was safe and we successfully initiated the turn, false if we did not turn.
     */
    private boolean turnTurretWithPower(double power)
    {
        boolean doTurn = armLevelSafe && elevatorLevelSafe;

        if (doTurn)
        {
            pidTurret.setPower(power);
        }

        return doTurn;
    }   //turnTurretWithPower

    /**
     * This method is called when the arm is done raising to the safe level. It will try to check if the elevator is
     * at safe level and turn the turret.
     *
     * @param context not used
     */
    private void armRaiseDoneCallback(Object context)
    {
        armLevelSafe = true;
        if (turnTurretToPos(turretTarget, turretPowerLimit, turretEvent, turretCallback))
        {
            turretTarget = 0.0;
            turretPowerLimit = 1.0;
            turretEvent = null;
            turretCallback = null;
        }
    }   //armRaiseDoneCallback

    /**
     * This method is called when the elevator is done raising to the safe level. It will try to check if the arm is
     * at safe level and turn the turret.
     *
     * @param context not used
     */
    private void elevatorRaiseDoneCallback(Object context)
    {
        elevatorLevelSafe = true;
        if (turnTurretToPos(turretTarget, turretPowerLimit, turretEvent, turretCallback))
        {
            turretTarget = 0.0;
            turretPowerLimit = 1.0;
            turretEvent = null;
            turretCallback = null;
        }
    }   //elevatorRaiseDoneCallback

}   //class Turret
