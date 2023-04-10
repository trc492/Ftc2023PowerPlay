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

package teamcode.autotasks;

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsysstems.BlinkinLEDs;

/**
 * This class implements auto-assist task to score a cone.
 */
public class TaskScoreCone extends TrcAutoTask<TaskScoreCone.State>
{
    private static final String moduleName = "TaskScoreCone";

    public enum State
    {
        START,
        GO_TO_START_POS,
        FIND_POLE,
        CHECK_FOR_POLE,
        RAISE_TO_SCORE_HEIGHT,
        EXTEND_ARM,
        CAP_POLE,
        SCORE_CONE,
        DONE
    }   //enum State

    private static class TaskParams
    {
        double startTarget;
        double startPowerLimit;
        double expectedTarget;
        double scoreHeight;
        double scanPower;
        double scanDuration;

        TaskParams(
            double startTarget, double startPowerLimit, double expectedTarget, double scoreHeight, double scanPower,
            double scanDuration)
        {
            this.startTarget = startTarget;
            this.startPowerLimit = startPowerLimit;
            this.expectedTarget = expectedTarget;
            this.scoreHeight = scoreHeight;
            this.scanPower = scanPower;
            this.scanDuration = scanDuration;
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;
    private String currOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     * @param msgTracer specifies the tracer to use to log events, can be null if not provided.
     */
    public TaskScoreCone(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.ownerName = ownerName;
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
     * @param expectedTarget specifies the expect turret position where the target should be.
     * @param scoreHeight specifies the elevator height to score the cone.
     * @param scanPower specifies how fast to scan for target, positive to scan left and negative to scan right.
     * @param scanDuration specifies how long to scan for target in seconds, scan will stop early if detected target.
     * @param event specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreCone(
        double startTarget, double startPowerLimit, double expectedTarget, double scoreHeight, double scanPower,
        double scanDuration, TrcEvent event)
    {
        final String funcName = "autoAssistScoreCone";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName,
                "%s: startTarget=%.2f, startPowerLimit=%.2f, expectedTarget=%.2f, scoreHeight=%.2f, scanPower=%.2f, " +
                "scanDuration=%.3f, event=%s",
                moduleName, startTarget, startPowerLimit, expectedTarget, scoreHeight, scanPower, scanDuration, event);
        }

        startAutoTask(
            State.START, new TaskParams(
                startTarget, startPowerLimit, expectedTarget, scoreHeight, scanPower, scanDuration), event);
    }   //autoAssistScoreCone

    /**
     * This method raises the elevator to the score height and starts scanning for the pole by slowly turning the
     * turret for a given maximum duration. If the pole is detected by the sensor, it will stop the turret, extend
     * the arm on top of the pole, lower the elevator to cap the pole, release the cone and retract the arm and
     * elevator.
     *
     * @param expectedTarget specifies the expect turret position where the target should be.
     * @param scoreHeight specifies the elevator height to score the cone.
     * @param scanPower specifies how fast to scan for target, positive to scan left and negative to scan right.
     * @param scanDuration specifies how long to scan for target in seconds, scan will stop early if detected target.
     * @param event specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreCone(
        double expectedTarget, double scoreHeight, double scanPower, double scanDuration, TrcEvent event)
    {
        autoAssistScoreCone(0.0, 0.0, expectedTarget, scoreHeight, scanPower, scanDuration, event);
    }   //autoAssistScoreCone

    /**
     * This method raises the elevator to the score height and starts scanning for the pole by slowly turning the
     * turret for a given maximum duration. If the pole is detected by the sensor, it will stop the turret, extend
     * the arm on top of the pole, lower the elevator to cap the pole, release the cone and retract the arm and
     * elevator.
     *
     * @param expectedTarget specifies the expect turret position where the target should be.
     * @param scanPower specifies how fast to scan for target, positive to scan left and negative to scan right.
     * @param scanDuration specifies how long to scan for target in seconds, scan will stop early if detected target.
     * @param event specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreCone(double expectedTarget, double scanPower, double scanDuration, TrcEvent event)
    {
        autoAssistScoreCone(0.0, 0.0, expectedTarget, 0.0, scanPower, scanDuration, event);
    }   //autoAssistScoreCone

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Canceling auto-assist score cone.", moduleName);
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
        boolean success = ownerName == null ||
                          (robot.turret.acquireExclusiveAccess(ownerName) &&
                           robot.elevator.acquireExclusiveAccess(ownerName) &&
                           robot.arm.acquireExclusiveAccess(ownerName));

        if (success)
        {
            currOwner = ownerName;
        }
        else
        {
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "%s: Failed to acquire subsystem ownership.", moduleName);
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

        if (ownerName != null)
        {
            if (msgTracer != null)
            {
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName, "%s: Releasing subsystem ownership (currOwner=%s, turret=%s, elevator=%s, arm=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.turret.getPidActuator()),
                    ownershipMgr.getOwner(robot.elevator), ownershipMgr.getOwner(robot.arm));
            }
            robot.turret.releaseExclusiveAccess(currOwner);
            robot.elevator.releaseExclusiveAccess(currOwner);
            robot.arm.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        robot.turret.cancel(currOwner);
        robot.elevator.cancel(currOwner);
        robot.arm.cancel(currOwner);
    }   //stopSubsystems.

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    protected void runTaskState(
        Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        final String funcName = "runTaskState";
        TaskParams taskParams = (TaskParams) params;
        //
        // Preconditions:
        // Arm is at up position, elevator is at min position, turret is optionally at start scan position.
        //
        switch (state)
        {
            case START:
                if (robot.arm.getPosition() > RobotParams.ARM_SAFE_POS)
                {
                    robot.arm.setPosition(currOwner, RobotParams.ARM_UP_POS, false, 1.0, event, 0.0);
                    sm.waitForSingleEvent(event, State.GO_TO_START_POS);
                    break;
                }
                else
                {
                    sm.setState(State.GO_TO_START_POS);
                }
                //
                // Intentionally falling through to the next state (GO_TO_START_POS).
                //
            case GO_TO_START_POS:
                if (taskParams.startPowerLimit != 0.0)
                {
                    // Turn the turret to the position for starting the scan.
                    robot.turret.setTarget(
                        currOwner, 0.0, taskParams.startTarget, false, taskParams.startPowerLimit, event, 5.0);
                    sm.waitForSingleEvent(event, State.FIND_POLE);
                    break;
                }
                else
                {
                    sm.setState(State.FIND_POLE);
                }
                //
                // Intentionally falling through to the next state (FIND_POLE).
                //
            case FIND_POLE:
                // Enable the sensor trigger and start scanning for the pole.
                // This operation takes about 0.5 sec.
                robot.turret.setTriggerEnabled(true);
                robot.turret.getPidActuator().setPower(currOwner, taskParams.scanPower, taskParams.scanDuration, event);
                sm.waitForSingleEvent(event, State.CHECK_FOR_POLE);
                break;

            case CHECK_FOR_POLE:
                if (robot.turret.detectedTarget())
                {
                    sm.setState(State.RAISE_TO_SCORE_HEIGHT);
                }
                else
                {
                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "Did not find the target (sensor=%.2f), go to default position.",
                            robot.turret.getSensorValue());
                    }
                    // We didn't see the target, let's go to expected position and hope it's there.
                    robot.turret.setTarget(
                        currOwner, 0.0, taskParams.expectedTarget, true, 0.5, event, 1.0);
                    sm.waitForSingleEvent(event, State.RAISE_TO_SCORE_HEIGHT);
                }
                break;

            case RAISE_TO_SCORE_HEIGHT:
                // Either found the pole or reached timeout. Either way, stop the turret.
                robot.turret.getPidActuator().setPower(currOwner, 0.0);
                robot.turret.setTriggerEnabled(false);

                if (taskParams.scoreHeight > 0.0)
                {
                    // Set the proper elevator height for scoring.
                    // This operation takes about 1.2 sec.
                    robot.elevator.setPosition(currOwner, 0.0, taskParams.scoreHeight, true, 1.0, event, 3.0);
                    sm.waitForSingleEvent(event, State.EXTEND_ARM);
                }
                else
                {
                    sm.setState(State.EXTEND_ARM);
                }
                break;

            case EXTEND_ARM:
                Double armTarget = null;
                if (robot.turret.detectedTarget())
                {
                    // Found the pole, set the LEDs to indicate so.
                    if (robot.blinkin != null)
                    {
                        robot.blinkin.setPatternState(BlinkinLEDs.GOT_YELLOW_POLE, true);
                    }
                    armTarget = robot.getScoringArmAngle();

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(funcName, "Found the pole (armAngle=%f).", armTarget);
                    }
                }
                // We are in autonomous mode or we detected target.
                if (armTarget == null || runMode == TrcRobot.RunMode.AUTO_MODE && armTarget > 60.0)
                {
                    // Can't determine a valid arm angle. Either we don't see the pole or the sensor is probably
                    // picking up a erroneous distance. In this case, just use the known good arm angle and hope it
                    // will work.
                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(funcName, "Failed to determine valid arm angle (armAngle=%f).", armTarget);
                    }
                    armTarget = 25.0;
                }
                // This operation takes about 1 sec.
                robot.arm.setPosition(currOwner, 0.0, armTarget, false, 1.0, event, 2.0);
                sm.waitForSingleEvent(event, State.CAP_POLE);
                break;

            case CAP_POLE:
                // Slowly set the elevator down to cap the pole to ensure secured scoring.
                robot.elevator.setPower(currOwner, -0.5, 0.2, event);
                sm.waitForSingleEvent(event, State.SCORE_CONE);
                break;

            case SCORE_CONE:
                // Release the cone to score it and retract the elevator and arm.
                robot.grabber.open();
                robot.arm.setPosition(currOwner, 0.5, RobotParams.ARM_UP_POS, false, 1.0, null, 0.0);
                robot.elevator.setPosition(currOwner, 0.5, RobotParams.ELEVATOR_MIN_POS, false, 1.0, null, 0.0);
                robot.turret.setTarget(currOwner, 0.5, RobotParams.TURRET_FRONT, true, 0.8, null, 0.0);
                //
                // Intentionally fall to the DONE state.
                //
            default:
            case DONE:
                robot.setGrabberAutoAssistOn(false);
                // Done, turn off the LEDs.
                if (robot.blinkin != null)
                {
                    robot.blinkin.setPatternState(BlinkinLEDs.GOT_YELLOW_POLE, false);
                }
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskScoreCone
