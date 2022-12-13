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
    public TaskPickupCone(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK, msgTracer);
        this.ownerName = ownerName;
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
     */
    public void autoAssistPickupCone(int conesRemaining, boolean useVision, TrcEvent event)
    {
        final String funcName = "autoAssistPickupCone";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "conesRemaining=%d, useVision=%s, event=%s, timeout=%.3f", conesRemaining, useVision, event);
        }

        startAutoTask(
            State.START,
            new TaskParams(conesRemaining, useVision && robot.vision != null && robot.vision.frontEocvVision != null),
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

        if (ownerName != null)
        {
            if (msgTracer != null)
            {
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName, "Releasing subsystem ownership (driveBase=%s, turret=%s, elevator=%s, arm=%s).",
                    ownershipMgr.getOwner(robot.robotDrive.driveBase), ownershipMgr.getOwner(robot.turret.getPidActuator()),
                    ownershipMgr.getOwner(robot.elevator), ownershipMgr.getOwner(robot.arm));
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
        robot.robotDrive.driveBase.stop(currOwner);
        robot.turret.cancel(currOwner);
        robot.elevator.cancel(currOwner);
        robot.arm.cancel(currOwner);
    }   //stopSubsystems.

    private TrcPose2D targetLocation;
    private Double expireTime;
    private boolean strafeTheOtherWay = false;

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     */
    @Override
    protected void runTaskState(Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
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
        //todo: for simplicity, this task is only tailored to auto, will adjust it to teleop if this works reliably for auto

        switch (state)
        {
            //prepares robot for using vision to pick up: turret to front, elevator down,
            //arm retracted, drive closer to conestack (auto only)
            case START:
                if (taskParams.useVision)
                {
                    robot.vision.frontEocvVision.setDetectObjectType(
                        FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            EocvVision.ObjectType.RED_CONE: EocvVision.ObjectType.BLUE_CONE);
                }
                targetLocation = null;
                //Todo(CodeReview): set arm to an angle that will let the distance sensor to see the cone stack.
                robot.arm.setTarget(currOwner, 20.0, false, 1.0, null, 0.0);
                //Todo(CodeReview): set elevator to min position so the distance sensor can see the cone stack.
                robot.elevator.setTarget(
                    currOwner, RobotParams.ELEVATOR_PICKUP_PRESETS[taskParams.conesRemaining], true, 1.0, null, 0.0);
                robot.turret.setTarget(currOwner, RobotParams.TURRET_FRONT, true, 0.8, event, 0.0);
                // Todo(CodeReview): precondition should be that the robot is already at the position to look for cone.
                // This will allow both autonomous and teleop to call it. The caller is responsible to get to this
                // position.
//                robot.robotDrive.purePursuitDrive.start(
//                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
//                    robot.robotDrive.getAutoTargetPoint(RobotParams.LOOK_FOR_CONE_POS_LEFT, FtcAuto.autoChoices));
                sm.waitForSingleEvent(event, State.LOOK_FOR_CONE);
                break;
            //if using vision, finds the cone
            case LOOK_FOR_CONE:
                if (taskParams.useVision)
                {
                    // Use vision to find the relative location of the cone.
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> coneInfo =
                        robot.vision.getDetectedConeInfo();
                    if (coneInfo != null)
                    {
                        targetLocation = new TrcPose2D(
                            coneInfo.distanceFromCamera.x, coneInfo.distanceFromCamera.y, coneInfo.horizontalAngle);
                    }
                }
                //if we found a target with vision, go to the next state
                if (targetLocation != null)
                {
                    sm.setState(State.DRIVE_TO_CONE);
                }
                //set expiretime if we are using vision but can't find the target
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
                        sm.setState(State.DRIVE_TO_CONE);
                    }
                }
                //if we are not using vision, go to next state
                else
                {
                    sm.setState(State.DRIVE_TO_CONE);
                }
                break;

            //Drive so the center of the robot is aligned with the cone
            case DRIVE_TO_CONE:
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                if (targetLocation != null)
                {
                    // Vision found the cone, drive to it with incremental pure pursuit.
                    // Use vision detect x but move forward with fixed distance and heading aligned to the field.
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(
                            targetLocation.x - 1.0, 5.0,
                            robot.robotDrive.getAutoTargetHeading(-90.0, FtcAuto.autoChoices) -
                            robot.robotDrive.driveBase.getHeading()));
                    sm.waitForSingleEvent(event, State.ALIGN_TO_CONE);
                }
                else
                {
                    // Either we did not use vision or vision did not detect anything. Use the absolute cone
                    // stack location.
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotDrive.getAutoTargetPoint(RobotParams.CONE_STACK_RED_LEFT, FtcAuto.autoChoices));
                    sm.waitForSingleEvent(event, State.PICKUP_CONE);
                }
                break;
            //Todo(CodeReview): why turning the drivebase? If using distance sensor, you have to raise the arm anyway.
            //I would use the distance sensor and the turret to scan for the cone and then lower the arm and elevator
            //to the correct position according to the sensor.
            //
            //And raising the arm will most likely clear the turret. You could use it to find the cones.
            //use drivebase to align because we can't turn turret for lower cone stacks (might hit the motor)
            //use turret/grabber sensor value to check if there is a cone in front
            //if we can't see it strave left for 2 seconds. if we still can't find it strafe the other way for 2 seconds(not implemented)
            case ALIGN_TO_CONE:
                //Todo(CodeReview):
                // 1. set turret to startScan position.
                // 2. start the scan.
                // 3. while distance is decreasing, keep scanning.
                // 4. stop scanning.
                // 5. lower arm and elevator to pickup position.
                // 6. turn on autoAssist grabber.
                // 7. go forward to grab it.
                if (robot.turret.detectedTarget())
                {
                    robot.robotDrive.cancel();
                    sm.setState(State.PICKUP_CONE);
                }
                //ran out of time, couldn't find pole stack
                else if(expireTime == null){
                    expireTime = TrcUtil.getCurrentTime() + 2;
                    robot.robotDrive.driveBase.holonomicDrive(currOwner, -0.1, 0, 0, 2, event);
                }
                else if(TrcUtil.getCurrentTime() == expireTime){
                    expireTime = null;
                    sm.setState(State.PICKUP_CONE);
                }
                break;

            case PICKUP_CONE: //2. lower elevator to the cone, wait for intake autoAssist
                robot.grabber.cancelAutoAssist();
                robot.robotDrive.driveBase.holonomicDrive(currOwner, 0.0, 0.2, 0.0);
                robot.grabber.enableAutoAssist(null, 0, event, 5);
                sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                break;

            case RAISE_ELEVATOR: //3 raise the elevator up higher than the pole
                robot.robotDrive.driveBase.stop(currOwner);
                robot.grabber.cancelAutoAssist();
                robot.grabber.close();
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                robot.robotDrive.driveBase.stop(currOwner);
                robot.elevator.setTarget(
                    currOwner, 0.5, RobotParams.HIGH_JUNCTION_SCORING_HEIGHT, true, 1.0, event, 4.0);
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
