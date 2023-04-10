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
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.vision.EocvVision;

/**
 * This class implements auto-assist task to pickup a cone.
 */
public class TaskPickupCone extends TrcAutoTask<TaskPickupCone.State>
{
    private static final String moduleName = "TaskPickupCone";
    private static final boolean useGrabberSensor = true;

    public enum State
    {
        START,
        LOOK_FOR_CONE,
        DRIVE_TO_CONE,
        APPROACH_CONE,
        PICKUP_CONE,
        LIFT_CONE,
        DONE
    }   //enum State

    private static class TaskParams
    {
        int conesRemaining;
        boolean useVision;

        TaskParams(int conesRemaining, boolean useVision)
        {
            this.conesRemaining = conesRemaining;
            this.useVision = useVision;
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent grabberEvent;
    private String currOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     * @param msgTracer specifies the tracer to use to log events, can be null if not provided.
     */
    public TaskPickupCone(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.ownerName = ownerName;
        this.robot = robot;
        this.msgTracer = msgTracer;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        grabberEvent = new TrcEvent(moduleName + ".grabberEvent");
    }   //TaskPickupCone

    /**
     * This method starts the auto-assist task to pick up a cone.
     *
     * @param conesRemaining specifies the number of remaining cones on the stack.
     * @param useVision specifies true to use vision, false otherwise.
     * @param event specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPickupCone(int conesRemaining, boolean useVision, TrcEvent event)
    {
        final String funcName = "autoAssistPickupCone";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "%s: conesRemaining=%d, useVision=%s, event=%s",
                moduleName, conesRemaining, useVision, event);
        }

        startAutoTask(
            State.START,
            new TaskParams(conesRemaining, useVision && robot.vision != null && robot.vision.eocvVision != null),
            event);
    }   //autoAssistPickupCone

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Canceling auto-assist pick up cone.", moduleName);
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
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName) &&
                           robot.turret.acquireExclusiveAccess(ownerName) &&
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
                    funcName,
                    "%s: Releasing subsystem ownership (currOwner=%s, driveBase=%s, turret=%s, elevator=%s, arm=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase),
                    ownershipMgr.getOwner(robot.turret.getPidActuator()), ownershipMgr.getOwner(robot.elevator),
                    ownershipMgr.getOwner(robot.arm));
            }
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
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
        final String funcName = "stopSubsystems";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Stopping subsystems.", moduleName);
        }
        robot.robotDrive.driveBase.stop(currOwner);
        robot.turret.cancel(currOwner);
        robot.elevator.cancel(currOwner);
        robot.arm.cancel(currOwner);
    }   //stopSubsystems.

    private TrcPose2D targetLocation;
    private Double expireTime;

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
        // Arm is at up position, elevator is at min position, turret is at front position.
        //
        switch (state)
        {
            case START:
                robot.globalTracer.traceInfo(funcName, "TURRET POS: %.1f", robot.turret.getPosition());
                if (taskParams.useVision)
                {
                    robot.vision.eocvVision.setDetectObjectType(
                        FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            EocvVision.ObjectType.RED_CONE: EocvVision.ObjectType.BLUE_CONE);
                }

                robot.turret.setTarget(currOwner, RobotParams.TURRET_FRONT, true, 0.8, null, 0.0);
                targetLocation = null;
                // Cancel grabber auto-assist just in case it was ON.
                robot.grabber.cancelAutoAssist();
                sm.setState(State.LOOK_FOR_CONE);
                //
                // Intentionally falling through to the next state (LOOK_FOR_CONE).
                //
            case LOOK_FOR_CONE:
                // This operation takes about 100 msec.
                robot.globalTracer.traceInfo(funcName, "TURRET POS: %.1f", robot.turret.getPosition());
                if (taskParams.useVision)
                {
                    // Use vision to find the relative location of the cone.
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> coneInfo =
                        robot.vision.getDetectedConeInfo();
                    if (coneInfo != null)
                    {
                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(funcName, "Found cone: coneInfo:%s", coneInfo);
                        }
                        targetLocation = new TrcPose2D(
                            coneInfo.distanceFromCamera.x - 1, coneInfo.distanceFromCamera.y, coneInfo.horizontalAngle);
                    }
                }

                if (!taskParams.useVision || targetLocation != null)
                {
                    // Either we are not using vision or Vision found a target. Either way, go to the next state.
                    sm.setState(State.DRIVE_TO_CONE);
                }
                else
                {
                    // We are using vision and vision did not detect anything, try again with timeout.
                    if (expireTime == null)
                    {
                        expireTime = TrcTimer.getCurrentTime() + 0.5;
                    }
                    else if (TrcTimer.getCurrentTime() >= expireTime)
                    {
                        // Times up, reset expireTime, give up and go to next state.
                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(funcName, "Ran out of time to look for cone, move on.");
                        }
                        expireTime = null;
                        sm.setState(State.DRIVE_TO_CONE);
                    }
                }
                break;

            case DRIVE_TO_CONE:
                robot.arm.setPosition(
                    currOwner, RobotParams.ARM_PICKUP_PRESETS[taskParams.conesRemaining], false, 1.0, null, 0.0);
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                if (targetLocation != null)
                {
                    // Vision found the cone, drive to it with incremental pure pursuit.
                    // Use vision detected x but not y. We will do y in APPROACH_CONE state.
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(
                            targetLocation.x, 0.0,
                            robot.robotDrive.getAutoTargetHeading(-90.0, FtcAuto.autoChoices) -
                            robot.robotDrive.driveBase.getHeading()));
                }
                else
                {
                    // Either we did not use vision or vision did not detect anything. Use the absolute cone
                    // stack location.
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotDrive.getAutoTargetPoint(RobotParams.CONE_STACK_RED_LEFT, FtcAuto.autoChoices));
                }
                sm.waitForSingleEvent(event, State.APPROACH_CONE);
                break;

            case APPROACH_CONE:
                double coneDist = RobotParams.GRABBER_DEF_CONE_DISTANCE;

                robot.grabber.enableAutoAssist(null, 0.0, grabberEvent, 0.0);
                sm.addEvent(grabberEvent);
                if (useGrabberSensor)
                {
                    double sensorDist = robot.grabber.getSensorValue();

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "grabberSensorDist=%.3f, turretPos=%.3f, conesRemaining=%d",
                            robot.grabber.getSensorValue(), robot.turret.getPosition(), taskParams.conesRemaining);
                    }

                    if (sensorDist <= 10.0)
                    {
                        coneDist = sensorDist;
                    }
                }
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                    new TrcPose2D(0.0, coneDist,
                        robot.robotDrive.getAutoTargetHeading(-90.0, FtcAuto.autoChoices) -
                        robot.robotDrive.driveBase.getHeading()));
                sm.addEvent(event);
                sm.waitForEvents(State.PICKUP_CONE);
                break;

            case PICKUP_CONE:
                robot.robotDrive.purePursuitDrive.cancel();
                if (!robot.grabber.hasObject())
                {
                    // Grabber sensor did not detect the cone. Turret may be slightly mis-aligned. Close the grabber
                    // anyway hoping it will grab it.
                    robot.grabber.close();
                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "Didn't detecte the cone, grab it anyway (grabberSensor=%.3f).",
                            robot.grabber.getSensorValue());
                    }
                }
                // Delay a little to make sure the grabber firmly grabbed the cone.
                timer.set(0.5, event);
                sm.waitForSingleEvent(event, State.LIFT_CONE);
                break;

            case LIFT_CONE:
                robot.arm.setPosition(currOwner, RobotParams.ARM_UP_POS, false, 1.0, event, 0.0);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            default:
            case DONE:
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskPickupCone
