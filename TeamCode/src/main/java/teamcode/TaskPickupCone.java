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
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * This class implements auto-assist task to pickup a cone.
 */
public class TaskPickupCone extends TrcAutoTask<TaskPickupCone.State>
{
    private static final String moduleName = "TaskPickupCone";

    public enum State
    {
        START,
        LOOK_FOR_CONE,
        DRIVE_TO_CONE,
        PREPARE_PICKUP,
        ALIGN_TO_CONE,
        ALIGN_OTHER_WAY,
        PICKUP_CONE,
        RAISE_ELEVATOR,
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

    private final String owner;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param owner specifies the owner ID to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     * @param msgTracer specifies the tracer to use to log events, can be null if not provided.
     */
    public TaskPickupCone(String owner, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, owner, TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK, msgTracer);
        this.owner = owner;
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName);
    }   //TaskPickupCone

    /**
     * This method starts the auto-assist task to pick up a cone.
     *
     * @param conesRemaining specifies the number of remaining cones on the stack.
     * @param useVision specifies true to use vision, false otherwise.
     * @param event specifies the event to signal when done, can be null if none provided.
     * @param timeout specifies the maximum amount of time for the completion of this operation in seconds.
     */
    public void autoAssistPickupCone(int conesRemaining, boolean useVision, TrcEvent event, double timeout)
    {
        final String funcName = "autoAssistPickupCone";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "conesRemaining=%d, useVision=%s, event=%s, timeout=%.3f",
                conesRemaining, useVision, event, timeout);
        }

        startAutoTask(
            State.START,
            new TaskParams(conesRemaining, useVision && robot.vision != null && robot.vision.frontEocvVision != null),
            event, timeout);
    }   //autoAssistPickupCone

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "Canceling auto-assist pick up cone.");
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
        boolean success = owner == null ||
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(owner) &&
                           robot.turret.acquireExclusiveAccess(owner) &&
                           robot.elevator.acquireExclusiveAccess(owner) &&
                           robot.arm.acquireExclusiveAccess(owner));

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

        if (owner != null)
        {
            if (msgTracer != null)
            {
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName, "Releasing subsystem ownership (driveBase=%s, turret=%s, elevator=%s, arm=%s).",
                    ownershipMgr.getOwner(robot.robotDrive.driveBase), ownershipMgr.getOwner(robot.turret.getPidActuator()),
                    ownershipMgr.getOwner(robot.elevator), ownershipMgr.getOwner(robot.arm));
            }
            robot.robotDrive.driveBase.releaseExclusiveAccess(owner);
            robot.turret.releaseExclusiveAccess(owner);
            robot.elevator.releaseExclusiveAccess(owner);
            robot.arm.releaseExclusiveAccess(owner);
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        robot.robotDrive.driveBase.stop(owner);
        robot.turret.cancel(owner);
        robot.elevator.cancel(owner);
        robot.arm.cancel(owner);
    }   //stopSubsystems.

    private TrcPose2D targetLocation;
    private Double expireTime;
    private boolean strafeTheOtherWay = false;

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
        //
        // 1. Prep subsystem to pick up cone: turn turret to front, lower elevator, raise arm (or should this be a
        //    precondition).
        // 2. Use vision to look for cone.
        // 3. Navigate robot to the cone, arm grabber autoAssist.
        // 4. Use the elevator sensor and/or grabber sensor to fine adjust the robot position and/or turret position
        //    to better aligned and grab the cone.
        // 5. Raise elevator.
        //
        switch (state)
        {
            case START:
                if (taskParams.useVision)
                {
                    robot.vision.frontEocvVision.setDetectObjectType(
                        FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            EocvVision.ObjectType.RED_CONE: EocvVision.ObjectType.BLUE_CONE);
                }
                //drive robot, orient turret, elevator arm so it can see the cone stack
                targetLocation = null;
                robot.arm.setTarget(owner, RobotParams.ARM_PICKUP_POS, false, 1.0, null, 0.0);
                robot.elevator.setTarget(owner, RobotParams.ELEVATOR_CONE_GRAB_HEIGHT, true, 1.0, null, 0.0);
                robot.turret.setTarget(owner, RobotParams.TURRET_FRONT, true, 0.8, event, 0.0);
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                robot.robotDrive.purePursuitDrive.start(
                    owner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                    robot.robotDrive.getAutoTargetPoint(RobotParams.LOOK_FOR_CONE_POS_LEFT, FtcAuto.autoChoices));
                sm.waitForSingleEvent(event, State.LOOK_FOR_CONE);
                break;
            //if using vision, finds the cone
            //if teleop, rotate counterclockwise until it sees the cone
            case LOOK_FOR_CONE:
                if (taskParams.useVision)
                {
                    // Use vision to find the relative location of the cone.
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> coneInfo =
                        robot.vision.getDetectedConeInfo();
                    if (coneInfo != null)
                    {
                        robot.robotDrive.driveBase.stop(owner);
                        targetLocation = new TrcPose2D(
                            coneInfo.distanceFromCamera.x - 1.0, 10,
                            coneInfo.horizontalAngle);
                    }
                }

                if (targetLocation != null)
                {
                    //use pure pursuit to align to the cone
                    robot.robotDrive.purePursuitDrive.start(
                        owner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0 , 0, targetLocation.angle));
                    sm.waitForSingleEvent(event, State.PREPARE_PICKUP);
                }
                else if (taskParams.useVision)
                {
                    // Vision did not detect anything, try again with timeout.
                    if (expireTime == null)
                    {
                        expireTime = TrcUtil.getCurrentTime() + 0.5;
                        //if we don't see the cone in teleop, rotate counterclockwise
                    }
                    else if (TrcUtil.getCurrentTime() >= expireTime)
                    {
                        // Times up, reset expireTime, go to next state.
                        expireTime = null;
                        sm.setState(State.PREPARE_PICKUP);
                    }
                }
                else
                {
                    sm.setState(State.PREPARE_PICKUP);
                }
                break;
            case PREPARE_PICKUP:
                robot.grabber.open();
                robot.arm.setTarget(owner, 20, false, 1.0, null, 0.0);
                robot.elevator.setTarget(
                    owner, RobotParams.ELEVATOR_PICKUP_PRESETS[taskParams.conesRemaining], true, 1.0, null, 0.0);
                robot.turret.setTarget(owner, RobotParams.TURRET_FRONT, true, 0.0, null, 0);
                sm.waitForSingleEvent(event, State.DRIVE_TO_CONE);
                break;
            case DRIVE_TO_CONE:
                if (targetLocation != null)
                {
                    // Vision found the cone, drive to it with incremental pure pursuit.
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotDrive.purePursuitDrive.start(
                            owner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(targetLocation.x, 5, FtcAuto.autoChoices.startPos == FtcAuto.StartPos.LEFT?
                                    270 - robot.robotDrive.driveBase.getHeading() : 90 - robot.robotDrive.driveBase.getHeading()));
                }
                else if(runMode != TrcRobot.RunMode.TELEOP_MODE)
                {
                    // Either we did not use vision or vision did not detect anything. Use the absolute cone
                    // stack location.
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotDrive.purePursuitDrive.start(
                            owner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.getAutoTargetPoint(RobotParams.CONE_STACK_RED_LEFT, FtcAuto.autoChoices));
                    sm.waitForSingleEvent(event, State.PICKUP_CONE);
                }
                else{
                    sm.setState(State.DONE);
                }
                break;
            //use drivebase to align because we can't turn turret for lower cone stacks (might hit the motor)
            //use turret/grabber sensor value to check if there is a cone in front
            //if we can't see it strave left for 2 seconds. if we still can't find it strafe the other way for 2 seconds(not implemented)
            //todo: replace this logic with something in turret to cancel pure pursuit when sensor sees cone
            case ALIGN_TO_CONE:
                if (robot.turret.getSensorValue() <= RobotParams.TURRET_SENSOR_PICKUP_THRESHOLD)
                {
                    robot.robotDrive.cancel();
                    sm.setState(State.PICKUP_CONE);
                }
                //ran out of time, couldn't find pole stack
                else if(expireTime == null){
                    expireTime = TrcUtil.getCurrentTime() + 2;
                    robot.robotDrive.driveBase.holonomicDrive(moduleName, -0.1, 0, 0, 2, event);
                }
                else if(TrcUtil.getCurrentTime() == expireTime){
                    expireTime = null;
                    sm.setState(State.PICKUP_CONE);
                }
                break;

            case PICKUP_CONE: //2. lower elevator to the cone, wait for intake autoAssist
                robot.grabber.cancelAutoAssist();
                robot.robotDrive.driveBase.holonomicDrive(owner, 0.0, 0.2, 0.0);
                robot.grabber.enableAutoAssist(null, 0, event, 5);
                sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                break;

            case RAISE_ELEVATOR: //3 raise the elevator up higher than the pole
                robot.robotDrive.driveBase.stop(owner);
                robot.grabber.cancelAutoAssist();
                robot.grabber.close();
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                robot.robotDrive.driveBase.stop(owner);
                robot.elevator.setTarget(
                    owner, 0.5, RobotParams.HIGH_JUNCTION_SCORING_HEIGHT, true, 1.0, event, 4.0);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            default:
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

    private void strafeRightToFindCone(Object context){
        strafeTheOtherWay = true;
    }

}   //class TaskPickupCone
