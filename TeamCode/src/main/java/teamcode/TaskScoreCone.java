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

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;

/**
 * This class implements auto-assist task to score a cone.
 */
public class TaskScoreCone extends TrcAutoTask<TaskScoreCone.State>
{
    private static final String moduleName = "TaskScoreCone";

    public enum State
    {
        TURN_TO_START_TARGET,
        RAISE_TO_SCORE_HEIGHT,
        FIND_POLE,
        EXTEND_ARM,
        CAP_POLE,
        SCORE_CONE,
        DONE
    }   //enum State

    private static class TaskParams
    {
        double startTarget;
        double startPowerLimit;
        double scoreHeight;
        double scanPower;
        double scanDuration;

        TaskParams(
            double startTarget, double startPowerLimit, double scoreHeight, double scanPower, double scanDuration)
        {
            this.startTarget = startTarget;
            this.startPowerLimit = startPowerLimit;
            this.scoreHeight = scoreHeight;
            this.scanPower = scanPower;
            this.scanDuration = scanDuration;
        }   //TaskParams
    }   //class TaskParams

    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     * @param msgTracer specifies the tracer to use to log events, can be null if not provided.
     */
    public TaskScoreCone(Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK, msgTracer);
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName);
    }   //TaskScoreCone

    /**
     * This method turns the turret to the start location, raises the elevator to the score height and starts scanning
     * for the pole by slowly turning the turret for a given maximum duration. If the pole is detected by the sensor,
     * it will stop the turret, extend the arm on top of the pole, lower the elevator to cap the pole, release the cone
     * and retract the arm and elevator.
     *
     * @param startTarget specifies the absolute start turret position to go to before doing the slow scan.
     * @param startPowerLimit specifies the turret power limit going to the startTarget.
     * @param scoreHeight specifies the elevator height to score the cone.
     * @param scanPower specifies how fast to scan for target, positive to scan left and negative to scan right.
     * @param scanDuration specifies how long to scan for target in seconds, scan will stop early if detected target.
     * @param event specifies the event to signal when done, can be null if none provided.
     * @param timeout specifies the maximum amount of time for the completion of this operation in seconds.
     */
    public void autoAssistScoreCone(
        double startTarget, double startPowerLimit, double scoreHeight, double scanPower, double scanDuration,
        TrcEvent event, double timeout)
    {
        final String funcName = "autoAssistScoreCone";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName,
                "startTarget=%.2f, startPowerLimit=%.2f, scoreHeight=%.2f, scanPower=%.2f, scanDuration=%.3f, " +
                "event=%s, timeout=%.3f",
                startTarget, startPowerLimit, scoreHeight, scanPower, scanDuration, event, timeout);
        }

        startAutoTask(
            startPowerLimit != 0.0?
                State.TURN_TO_START_TARGET:
                scoreHeight > 0.0? State.RAISE_TO_SCORE_HEIGHT: State.FIND_POLE,
            new TaskParams(startTarget, startPowerLimit, scoreHeight, scanPower, scanDuration),
            event, timeout);
    }   //autoAssistScoreCone

    /**
     * This method raises the elevator to the score height and starts scanning for the pole by slowly turning the
     * turret for a given maximum duration. If the pole is detected by the sensor, it will stop the turret, extend
     * the arm on top of the pole, lower the elevator to cap the pole, release the cone and retract the arm and
     * elevator.
     *
     * @param scoreHeight specifies the elevator height to score the cone.
     * @param scanPower specifies how fast to scan for target, positive to scan left and negative to scan right.
     * @param scanDuration specifies how long to scan for target in seconds, scan will stop early if detected target.
     * @param event specifies the event to signal when done, can be null if none provided.
     * @param timeout specifies the maximum amount of time for the completion of this operation in seconds.
     */
    public void autoAssistScoreCone(
        double scoreHeight, double scanPower, double scanDuration, TrcEvent event, double timeout)
    {
        autoAssistScoreCone(0.0, 0.0, scoreHeight, scanPower, scanDuration, event, timeout);
    }   //autoAssistScoreCone

    /**
     * This method raises the elevator to the score height and starts scanning for the pole by slowly turning the
     * turret for a given maximum duration. If the pole is detected by the sensor, it will stop the turret, extend
     * the arm on top of the pole, lower the elevator to cap the pole, release the cone and retract the arm and
     * elevator.
     *
     * @param scanPower specifies how fast to scan for target, positive to scan left and negative to scan right.
     * @param scanDuration specifies how long to scan for target in seconds, scan will stop early if detected target.
     * @param event specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreCone(double scanPower, double scanDuration, TrcEvent event)
    {
        autoAssistScoreCone(0.0, 0.0, 0.0, scanPower, scanDuration, event, 0.0);
    }   //autoAssistScoreCone

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "Canceling auto-assist score cone.");
        }

        stopAutoTask(false);
    }   //autoAssistCancel

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
     * operation. This is typically done before starting an auto-assist operation.
     *
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership()
    {
        final String funcName = "acquireSubsystemsOwnership";
        boolean success = robot.turret.acquireExclusiveAccess(moduleName) &&
                          robot.elevator.acquireExclusiveAccess(moduleName) &&
                          robot.arm.acquireExclusiveAccess(moduleName);

        if (!success)
        {
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "Failed to acquire subsystem ownership.");
            }
            releaseSubsystemsOwnership();
        }

        return success;
    }   //acquireSubsystemsOwnership

    /**
     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
     * operation. This is typically done if the auto-assist operation is completed or canceled.
     */
    @Override
    protected void releaseSubsystemsOwnership()
    {
        final String funcName = "releaseSubsystemsOwnership";

        if (msgTracer != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            msgTracer.traceInfo(
                funcName, "Releasing subsystem ownership (turret=%s, elevator=%s, arm=%s).",
                ownershipMgr.getOwner(robot.turret.getPidActuator()), ownershipMgr.getOwner(robot.elevator),
                ownershipMgr.getOwner(robot.arm));
        }
        robot.turret.releaseExclusiveAccess(moduleName);
        robot.elevator.releaseExclusiveAccess(moduleName);
        robot.arm.releaseExclusiveAccess(moduleName);
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        robot.turret.cancel(moduleName);
        robot.elevator.cancel(moduleName);
        robot.arm.cancel(moduleName);
    }   //stopSubsystems.

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param timeout specifies the timeout for executing the current state, can be zero if no timeout.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     */
    @Override
    protected void runTaskState(
        Object params, State state, double timeout, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "runTaskState";
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case TURN_TO_START_TARGET:
                robot.turret.setTarget(
                    moduleName, 0.0, taskParams.startTarget, false, taskParams.startPowerLimit,
                    event, 5.0);
                sm.waitForSingleEvent(event, State.RAISE_TO_SCORE_HEIGHT);
                break;

            case RAISE_TO_SCORE_HEIGHT:
                robot.elevator.setTarget(
                    moduleName, 0.0, taskParams.scoreHeight, true, 1.0, event, 3);
                sm.waitForSingleEvent(event, State.FIND_POLE);
                break;

            case FIND_POLE:
                robot.turret.setTriggerEnabled(true);
                robot.turret.getPidActuator().setPower(
                    moduleName, taskParams.scanPower, taskParams.scanDuration, event);
                sm.waitForSingleEvent(event, State.EXTEND_ARM);
                break;

            case EXTEND_ARM:
                robot.turret.getPidActuator().setPower(moduleName, 0.0);
                robot.turret.setTriggerEnabled(false);
                if (robot.turret.detectedPole())
                {
                    double armTarget = robot.getScoringArmAngle();
                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(funcName, "armTarget=%.1f", armTarget);
                    }
                    robot.arm.setTarget(moduleName, 0.0, armTarget, false, 1.0, event, 0.0);
                    sm.waitForSingleEvent(event, State.CAP_POLE);
                }
                else
                {
                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "Failed to find pole (sensor=%.2f).", robot.turret.getSensorValue());
                    }
                    sm.setState(State.DONE);
                }
                break;

            case CAP_POLE:
                robot.elevator.setPower(moduleName, -0.2, 0.5, event);
                sm.waitForSingleEvent(event, State.SCORE_CONE);
                break;

            case SCORE_CONE:
                robot.grabber.cancelAutoAssist();
                robot.arm.setTarget(moduleName, RobotParams.ARM_MIN_POS, false, 1.0, null, 0.0);
                robot.elevator.setTarget(moduleName, RobotParams.ELEVATOR_MIN_POS, false, 1.0, null, 0.0);
                //
                // Intentionally fall to the DONE state.
                //
            default:
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskScoreCone
