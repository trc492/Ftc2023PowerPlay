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
        APPROACH_CONE,
        PICKUP_CONE,
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
                funcName, "%s: conesRemaining=%d, useVision=%s, event=%s",
                moduleName, conesRemaining, useVision, event);
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
     */
    @Override
    protected void runTaskState(Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
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
                if (taskParams.useVision)
                {
                    robot.vision.frontEocvVision.setDetectObjectType(
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
                if (taskParams.useVision)
                {
                    // Use vision to find the relative location of the cone.
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> coneInfo =
                        robot.vision.getDetectedConeInfo();
                    if (coneInfo != null)
                    {
                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(funcName, "Found cone: coneInfo=%s", coneInfo);
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
                        expireTime = TrcUtil.getCurrentTime() + 0.5;
                    }
                    else if (TrcUtil.getCurrentTime() >= expireTime)
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
                // Note: both grabber and PurePursuit are signaling the same event, so either one will move us to the
                // next state.
                robot.arm.setTarget(
                    currOwner, RobotParams.ARM_PICKUP_PRESETS[taskParams.conesRemaining], false, 1.0, null, 0.0);
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                if (targetLocation != null)
                {
                    // Vision found the cone, drive to it with incremental pure pursuit.
                    // Use vision detected x but move forward with fixed distance and heading aligned to the field.
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(
                            targetLocation.x, 0,
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
                robot.grabber.enableAutoAssist(null, 0.0, event, 0.0);
                robot.robotDrive.driveBase.holonomicDrive(currOwner, 0.0, 0.15, 0.0, 2.5, event);
                sm.waitForSingleEvent(event, State.PICKUP_CONE);
                break;

            case PICKUP_CONE:
                // Have the cone or not, we will clean up: stop PurePursuit, stop grabber, raise elevator, retract arm.
                robot.robotDrive.driveBase.stop(currOwner);
                robot.robotDrive.purePursuitDrive.cancel();
                robot.elevator.setTarget(currOwner, RobotParams.ELEVATOR_MIN_POS + 12.0, false, 1.0, event, 0.0);
                robot.arm.setTarget(currOwner, 0.5, RobotParams.ARM_UP_POS, false, 1.0, null, 0.0);
                if (!robot.grabber.hasObject())
                {
                    // We don't have the cone. What to do? Maybe just go to DONE anyway?
                    // If we have time, we may do the fancy sensor scanning here.
                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(funcName, "Don't have the cone, giving up.");
                    }
                }
                sm.waitForSingleEvent(event, State.DONE);
                break;

//            case SET_TURRET_SCAN_POS:
//                if(targetLocation.x < 0 ){
//                    //scan left with
//                    robot.turret.setTriggerEnabled(true);
//                    robot.turret.getPidActuator().setPower(currOwner, RobotParams.TURRET_SCAN_POWER, RobotParams.TURRET_SCAN_DURATION, event);
//                }
//                break;
//            //Todo(CodeReview): why turning the drivebase? If using distance sensor, you have to raise the arm anyway.
//            //I would use the distance sensor and the turret to scan for the cone and then lower the arm and elevator
//            //to the correct position according to the sensor.
//            //
//            //And raising the arm will most likely clear the turret. You could use it to find the cones.
//            //use drivebase to align because we can't turn turret for lower cone stacks (might hit the motor)
//            //use turret/grabber sensor value to check if there is a cone in front
//            //if we can't see it strave left for 2 seconds. if we still can't find it strafe the other way for 2 seconds(not implemented)
//            case ALIGN_TO_CONE:
//                //set two angles of possibiliy
//                //Todo(CodeReview):
//                // 1. set turret to startScan position.
//                // 2. start the scan.
//                // 3. while distance is decreasing, keep scanning.
//                // 4. stop scanning.
//                // 5. lower arm and elevator to pickup position.
//                // 6. turn on autoAssist grabber.
//                // 7. go forward to grab it.
//                if (robot.turret.detectedTarget())
//                {
//                    robot.robotDrive.cancel();
//                    sm.setState(State.PICKUP_CONE);
//                }
//                //ran out of time, couldn't find pole stack
//                else if(expireTime == null){
//                    expireTime = TrcUtil.getCurrentTime() + 2;
//                    robot.robotDrive.driveBase.holonomicDrive(currOwner, -0.1, 0, 0, 2, event);
//                }
//                else if(TrcUtil.getCurrentTime() == expireTime){
//                    expireTime = null;
//                    sm.setState(State.PICKUP_CONE);
//                }
//                break;

            default:
            case DONE:
                robot.robotDrive.driveBase.stop(currOwner);
                robot.elevator.setTarget(currOwner, RobotParams.ELEVATOR_MIN_POS, false, 1.0, null, 0.0);
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskPickupCone
