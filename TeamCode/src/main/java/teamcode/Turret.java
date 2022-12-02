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
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcFtcLib.ftclib.FtcDigitalInput;
import TrcFtcLib.ftclib.FtcMotorActuator;

/**
 * This class encapsulates a PID Actuator as the Turret. Since the turret is not safe to swing around unless the
 * intake is raised to a safe height so that it won't hit the drive base motors and electronics, we will provide
 * the overriding methods of setPower and setTarget that will make sure the intake is raised above "safe level"
 * before turning the turret.
 */
public class Turret implements TrcExclusiveSubsystem
{
    private static final boolean debugEnabled = false;

    private enum State
    {
        START,
        CHECK_SAFETY,
        TURN_TURRET,
        DO_POST_OP,
        DONE
    }   //enum State

    private static class ActionParams
    {
        double delay;
        double target;
        double powerLimit;
        TrcEvent event;
        double timeout;
        Double elevatorTarget;
        Double armTarget;
        boolean isSetPower;
    }   //class ActionParams

    private static final String moduleName = "Turret";
    private final ActionParams actionParams = new ActionParams();
    private final Robot robot;
    private final FtcDigitalInput calDirectionSwitch;
    private final TrcPidActuator pidTurret;
    private final TrcEvent armEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject turretTaskObj;
    private double prevTurretPower = 0.0;

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
            .resetPositionOnLowerLimit(false)
            .setPidParams(new TrcPidController.PidParameters(
                RobotParams.TURRET_KP, RobotParams.TURRET_KI, RobotParams.TURRET_KD,
                RobotParams.TURRET_TOLERANCE))
            .setPresetTolerance(RobotParams.TURRET_PRESET_TOLERANCE)
            .setPosPresets(RobotParams.TURRET_PRESET_LEVELS);
        pidTurret = new FtcMotorActuator(
            RobotParams.HWNAME_TURRET, motorParams, turretParams).getPidActuator();
        pidTurret.setMsgTracer(robot.globalTracer);
        armEvent = new TrcEvent(moduleName + ".armEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        turretTaskObj = TrcTaskMgr.createTask("turretTask", this::turretTask);
    }   //Turret

    /**
     * This method returns the PID actuator object.
     *
     * @return PID Actuator object.
     */
    public TrcPidActuator getPidActuator()
    {
        return pidTurret;
    }   //getPidActuator

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
        event.setCallback(this::armZeroCalDoneCallback, null);
        robot.arm.zeroCalibrate(RobotParams.ARM_CAL_POWER, event);
    }   //zeroCalibrate

    /**
     * This method is called after the arm zero calibration is done so that we know it's safe to zero calibrate the
     * turret.
     *
     * @param context not used.
     */
    private void armZeroCalDoneCallback(Object context)
    {
        double calPower = Math.abs(RobotParams.TURRET_CAL_POWER);
        pidTurret.zeroCalibrate(calDirectionSwitch.isActive()? calPower: -calPower);
    }   //armZeroCalDoneCallback

    /**
     * This method sets the turret to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     */
    public void setPresetPosition(int preset)
    {
        if (pidTurret.validatePresetIndex(preset))
        {
            setTarget(RobotParams.TURRET_PRESET_LEVELS[preset], 1.0, null, 0.0, null, null);
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
     * This method check if a turret operation is in progress.
     *
     * @return true if a turret operation is in progress, false otherwise.
     */
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels a turret operation if one is in progress.
     */
    public void cancel()
    {
        if (sm.isEnabled())
        {
            setTaskEnabled(false);
            pidTurret.cancel();
            robot.arm.releaseExclusiveAccess(moduleName);
            robot.elevator.releaseExclusiveAccess(moduleName);
        }
    }   //cancel

    /**
     * This method enables/disables the turret task.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        boolean active = sm.isEnabled();

        if (!active && enabled)
        {
            sm.start(State.START);
            turretTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
        }
        else if (active && !enabled)
        {
            turretTaskObj.unregisterTask();
            sm.stop();
        }
    }   //setTaskEnabled

    /**
     * This method sets the turret target position. It first checks if it's safe for the turret to turn without
     * hitting anything. If it's not safe, it will first raise the arm above the "safe level" before setting the
     * target for the turret.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before performing the action.
     * @param target specifies the target position of the turret in degrees.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param timeout specifies timeout in seconds for the operation.
     * @param elevatorTarget specifies optionally the elevator target, can be null if no additional elevator movement.
     * @param armTarget specifies optionally the arm target, can be null if no additional arm movement.
     */
    public void setTarget(
        String owner, double delay, double target, double powerLimit, TrcEvent event, double timeout,
        Double elevatorTarget, Double armTarget)
    {
        if (validateOwnership(owner))
        {
            actionParams.delay = delay;
            actionParams.target = target;
            actionParams.powerLimit = powerLimit;
            actionParams.event = event;
            actionParams.timeout = timeout;
            actionParams.elevatorTarget = elevatorTarget;
            actionParams.armTarget = armTarget;
            actionParams.isSetPower = false;
            setTaskEnabled(true);
        }
    }   //setTarget

    /**
     * This method sets the turret target position. It first checks if it's safe for the turret to turn without
     * hitting anything. If it's not safe, it will first raise the arm above the "safe level" before setting the
     * target for the turret.
     *
     * @param delay specifies the delay in seconds before performing the action.
     * @param target specifies the target position of the turret in degrees.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param timeout specifies timeout in seconds for the operation.
     * @param elevatorTarget specifies optionally the elevator target, can be null if no additional elevator movement.
     * @param armTarget specifies optionally the arm target, can be null if no additional arm movement.
     */
    public void setTarget(
        double delay, double target, double powerLimit, TrcEvent event, double timeout,
        Double elevatorTarget, Double armTarget)
    {
        setTarget(null, delay, target, powerLimit, event, timeout, elevatorTarget, armTarget);
    }   //setTarget

    /**
     * This method sets the turret target position. It first checks if it's safe for the turret to turn without
     * hitting anything. If it's not safe, it will first raise the arm above the "safe level" before setting the
     * target for the turret.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param timeout specifies timeout in seconds for the operation.
     * @param elevatorTarget specifies optionally the elevator target, can be null if no additional elevator movement.
     * @param armTarget specifies optionally the arm target, can be null if no additional arm movement.
     */
    public void setTarget(
        double target, double powerLimit, TrcEvent event, double timeout, Double elevatorTarget, Double armTarget)
    {
        setTarget(null, 0.0, target, powerLimit, event, timeout, elevatorTarget, armTarget);
    }   //setTarget

    /**
     * This method sets the turret target position. It first checks if it's safe for the turret to turn without
     * hitting anything. If it's not safe, it will first raise the arm above the "safe level" before setting the
     * target for the turret.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     * @param timeout specifies timeout in seconds for the operation.
     */
    public void setTarget(double target, double powerLimit, TrcEvent event, double timeout)
    {
        setTarget(null, 0.0, target, powerLimit, event, timeout, null, null);
    }   //setTarget

    /**
     * This method sets the turret target position. It first checks if it's safe for the turret to turn without
     * hitting anything. If it's not safe, it will first raise the arm above the "safe level" before setting the
     * target for the turret.
     *
     * @param target specifies the target position of the turret in degrees.
     * @param powerLimit specifies the maximum power the turret will turn.
     * @param event specifies the event to signal when the turret is on target, can be null if not provided.
     */
    public void setTarget(double target, double powerLimit, TrcEvent event)
    {
        setTarget(null, 0.0, target, powerLimit, event, 0.0, null, null);
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
        setTarget(null, 0.0, target, powerLimit, null, 0.0, null, null);
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
        setTarget(null, 0.0, target, 1.0, null, 0.0, null, null);
    }   //setTarget

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
        if (validateOwnership(owner))
        {
            if (power != prevTurretPower)
            {
                if (power == 0.0)
                {
                    cancel();
                    pidTurret.setPidPower(power, false);
                    prevTurretPower = power;
                }
                else if (!isActive())
                {
                    if (isArmLevelSafe() && isElevatorLevelSafe())
                    {
                        if (usePid)
                        {
                            pidTurret.setPidPower(power);
                        }
                        else
                        {
                            pidTurret.setPower(power);
                        }
                        prevTurretPower = power;
                    }
                    else
                    {
                        actionParams.isSetPower = true;
                        setTaskEnabled(true);
                    }
                }
            }
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
     * This method is called periodically to perform the specified turret task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    private void turretTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "turretTask";
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            boolean waitForArmOrElevator = false;

            switch (state)
            {
                case START:
                    if (!actionParams.isSetPower && actionParams.delay > 0.0)
                    {
                        if (debugEnabled)
                        {
                            robot.globalTracer.traceInfo(funcName, "Doing delay=%.3f", actionParams.delay);
                        }
                        timer.set(actionParams.delay, event);
                        sm.waitForSingleEvent(event, State.CHECK_SAFETY);
                        break;
                    }
                    else
                    {
                        sm.setState(State.CHECK_SAFETY);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }

                case CHECK_SAFETY:
                    boolean abort = false;

                    if (!isArmLevelSafe())
                    {
                        if (robot.arm.acquireExclusiveAccess(moduleName))
                        {
                            if (debugEnabled)
                            {
                                robot.globalTracer.traceInfo(funcName, "Moving arm to safe level.");
                            }
                            robot.arm.setTarget(
                                moduleName, RobotParams.ARM_MIN_POS_FOR_TURRET, false, 1.0, armEvent, 0.0);
                            sm.addEvent(armEvent);
                            waitForArmOrElevator = true;
                        }
                        else
                        {
                            robot.globalTracer.traceWarn(
                                funcName, "Failed to acquire arm ownership (armOwner=%s).",
                                TrcOwnershipMgr.getInstance().getOwner(robot.arm));
                            abort = true;
                        }
                    }

                    if (!isElevatorLevelSafe())
                    {
                        if (robot.elevator.acquireExclusiveAccess(moduleName))
                        {
                            if (debugEnabled)
                            {
                                robot.globalTracer.traceInfo(funcName, "Moving elevator to safe level.");
                            }
                            robot.elevator.setTarget(
                                moduleName, RobotParams.ELEVATOR_MIN_POS_FOR_TURRET + 2.0, true, 1.0, elevatorEvent,
                                0.25);
                            sm.addEvent(elevatorEvent);
                            waitForArmOrElevator = true;
                        }
                        else
                        {
                            robot.globalTracer.traceWarn(
                                funcName, "Failed to acquire elevator ownership (elevatorOwner=%s).",
                                TrcOwnershipMgr.getInstance().getOwner(robot.elevator));
                            abort = true;
                        }
                    }

                    if (waitForArmOrElevator)
                    {
                        sm.waitForEvents(State.TURN_TURRET);
                    }
                    else
                    {
                        sm.setState(abort? State.DONE: State.TURN_TURRET);
                    }
                    break;

                case TURN_TURRET:
                    robot.arm.releaseExclusiveAccess(moduleName);
                    robot.elevator.releaseExclusiveAccess(moduleName);
                    if (!actionParams.isSetPower)
                    {
                        if (debugEnabled)
                        {
                            robot.globalTracer.traceInfo(
                                funcName, "Turning turret to position %.1f.", actionParams.target);
                        }
                        pidTurret.setTarget(actionParams.target, true, 1.0, event);
                        sm.waitForSingleEvent(event, State.DO_POST_OP);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DO_POST_OP:
                    prevTurretPower = 0.0;
                    if (actionParams.armTarget != null)
                    {
                        if (debugEnabled)
                        {
                            robot.globalTracer.traceInfo(
                                funcName, "Setting arm to post-op position %.1f.", actionParams.armTarget);
                        }
                        robot.arm.setTarget(actionParams.armTarget, false, 1.0, armEvent);
                        sm.addEvent(armEvent);
                        waitForArmOrElevator = true;
                    }

                    if (actionParams.elevatorTarget != null)
                    {
                        if (debugEnabled)
                        {
                            robot.globalTracer.traceInfo(
                                funcName, "Setting elevator to post-op position %.1f.", actionParams.elevatorTarget);
                        }
                        robot.elevator.setTarget(actionParams.elevatorTarget, true, 1.0, elevatorEvent);
                        sm.addEvent(elevatorEvent);
                        waitForArmOrElevator = true;
                    }

                    if (waitForArmOrElevator)
                    {
                        sm.waitForEvents(State.DONE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                default:
                case DONE:
                    cancel();
                    if (actionParams.event != null)
                    {
                        actionParams.event.signal();
                    }
                    break;
            }

            if (debugEnabled)
            {
                robot.globalTracer.traceStateInfo(sm.toString(), sm.getState(), null, null, null, null);
            }
        }
    }   //turretTask

    /**
     * This method checks if the arm level is safe for the turret turn.
     *
     * @return true if it's safe for the turret to turn without hitting things, false otherwise.
     */
    private boolean isArmLevelSafe()
    {
        return robot.arm.getPosition() <= RobotParams.ARM_MIN_POS_FOR_TURRET;
    }   //isArmLevelSafe

    /**
     * This method checks if the elevator level is safe for the turret turn.
     *
     * @return true if it's safe for the turret to turn without hitting things, false otherwise.
     */
    private boolean isElevatorLevelSafe()
    {
        return robot.elevator.getPosition() >= RobotParams.ELEVATOR_MIN_POS_FOR_TURRET;
    }   //isElevatorLevelSafe

}   //class Turret
