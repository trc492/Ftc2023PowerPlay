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
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * This class implements auto-assist task to pickup a cone.
 */
public class TaskPickupCone
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
    }

    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject autoAssistTaskObj;
    private TrcEvent onFinishEvent;
    private Double expireTime;
    private int conesRemaining;
    private boolean useVision;
    private TrcPose2D targetLocation;
    boolean strafeTheOtherWay = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskPickupCone(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        autoAssistTaskObj = TrcTaskMgr.createTask(moduleName, this::autoAssistTask);
    }   //TaskPickupCone

    /**
     * This method cancels the auto-assist operation in progress if any.
     */
    public void cancel()
    {
        stop();
        if (onFinishEvent != null)
        {
            onFinishEvent.cancel();
            onFinishEvent = null;
        }
    }   //cancel

    /**
     * This method starts the auto-assist task to pick up a cone.
     *
     * @param event specifies the event to signal when the auto-assist task is done, can be null if not provided.
     * @param timeout specifies the maximum time in seconds for the operation, can be zero if no timeout.
     */
    public void autoAssistPickupCone(TrcEvent event, double timeout, int conesRemaining, boolean useVision)
    {
        this.useVision = (robot.vision != null && robot.vision.frontEocvVision != null)?
                useVision : false;
        this.conesRemaining = conesRemaining;
        this.onFinishEvent = event;
        this.expireTime = timeout > 0.0? TrcUtil.getCurrentTime() + timeout: 0.0;
        setTaskEnabled(true);
    }   //autoAssistPickupCone

    /**
     * This method stops the auto-assist operation in progress if any.
     */
    private void stop()
    {
        setTaskEnabled(false);
        robot.arm.cancel();
        robot.elevator.cancel();
        robot.turret.cancel();
    }   //stop

    /**
     * This method enables/disables the auto-assist task.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled && !sm.isEnabled())
        {
            sm.start(State.START);
            autoAssistTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
        }
        else if (!enabled && sm.isEnabled())
        {
            sm.stop();
            autoAssistTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     */
    private void autoAssistTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "autoAssistTask";
        State state = sm.checkReadyAndGetState();
        //
        // 1. Prep subsystem to pick up cone: turn turret to front, lower elevator, raise arm (or should this be a
        //    precondition).
        // 2. Use vision to look for cone.
        // 3. Navigate robot to the cone, arm grabber autoAssist.
        // 4. Use the elevator sensor and/or grabber sensor to fine adjust the robot position and/or turret position
        //    to better aligned and grab the cone.
        // 5. Raise elevator.
        //
        if (state != null)
        {
            switch (state)
            {
                case START:
                    if (useVision)
                    {
                        robot.vision.frontEocvVision.setDetectObjectType(
                                FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                                        EocvVision.ObjectType.RED_CONE: EocvVision.ObjectType.BLUE_CONE);
                    }
                    //drive robot, orient turret, elevator arm so it can see the cone stack
                    targetLocation = null;
                    robot.arm.setTarget(RobotParams.ARM_PICKUP_POS);
                    robot.elevator.setTarget(RobotParams.ELEVATOR_CONE_GRAB_HEIGHT, true);
                    robot.turret.setTarget(RobotParams.TURRET_FRONT, true, 0.8, event, 0.0);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.getAutoTargetPoint(RobotParams.LOOK_FOR_CONE_POS_LEFT, FtcAuto.autoChoices));
                    sm.waitForSingleEvent(event, State.LOOK_FOR_CONE);
                    break;
                //if using vision, finds the cone
                //if teleop, rotate counterclockwise until it sees the cone
                case LOOK_FOR_CONE:
                    if (useVision)
                    {
                        // Use vision to find the relative location of the cone.
                        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> coneInfo =
                                robot.vision.getDetectedConeInfo();
                        if (coneInfo != null)
                        {
                            robot.robotDrive.driveBase.stop();
                            targetLocation = new TrcPose2D(
                                    coneInfo.distanceFromCamera.x - 1.0, 10,
                                    coneInfo.horizontalAngle);
                        }
                    }
                    if (targetLocation != null)
                    {
                        //use pure pursuit to align to the cone
                            robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), true,
                                    new TrcPose2D(0 , 0, targetLocation.angle));
                            sm.waitForSingleEvent(event, State.PREPARE_PICKUP);

                    }
                    else if (useVision)
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
                    robot.arm.setTarget(RobotParams.ARM_PICKUP_POS);
                    robot.elevator.setTarget(RobotParams.ELEVATOR_PICKUP_PRESETS[conesRemaining]);
                    robot.turret.setTarget(
                            0.0, RobotParams.TURRET_FRONT, true, 0.0,
                            null, 0);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_CONE);
                    break;
                case DRIVE_TO_CONE:
                    if (targetLocation != null)
                    {
                        // Vision found the cone, drive to it with incremental pure pursuit.
                        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                        robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), true,
                                new TrcPose2D(targetLocation.x, 5,270 - robot.robotDrive.driveBase.getHeading()));
                    }
                    else if(runMode != TrcRobot.RunMode.TELEOP_MODE)
                    {
                        // Either we did not use vision or vision did not detect anything. Use the absolute cone
                        // stack location.
                        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                        robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.getAutoTargetPoint(RobotParams.CONE_STACK_RED_LEFT, FtcAuto.autoChoices));
                    }
                    else{
                        sm.setState(State.DONE);
                    }
                    sm.waitForSingleEvent(event, State.PICKUP_CONE);
                    break;
                //use drivebase to align because we can't turn turret for lower cone stacks (might hit the motor)
                //use turret/grabber sensor value to check if there is a cone in front
                //if we can't see it strave left for 2 seconds. if we still can't find it strafe the other way for 2 seconds(not implemented)
                //todo: replace this logic with something in turret to cancel pure pursuit when sensor sees cone
                case ALIGN_TO_CONE:
                    if(robot.turret.getSensorValue() <= RobotParams.TURRET_SENSOR_PICKUP_THRESHOLD){
                        robot.robotDrive.cancel();
                        sm.setState(State.PICKUP_CONE);
                    }
                    //ran out of time, couldn't find pole stack
                    else if(TrcUtil.getCurrentTime() == expireTime){
                        expireTime = null;
                        sm.setState(State.DONE);
                    }
                    else{
                        if(expireTime == null){
                            expireTime = TrcUtil.getCurrentTime() + 2;
                        }
                        robot.robotDrive.driveBase.holonomicDrive(-0.1, 0, 0, 2, event);
                    }
                    break;


                case PICKUP_CONE: //2. lower elevator to the cone, wait for intake autoAssist
                    TrcEvent event2 = new TrcEvent("event2");
                    event2.setCallback(this::grabberCancelDrive, null);
                    robot.grabber.cancelAutoAssist();
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.2, 0.0);
                    robot.grabber.enableAutoAssist(null, 0, event2, 5);
                    sm.waitForSingleEvent(event2, State.RAISE_ELEVATOR);
                    break;

                case RAISE_ELEVATOR: //3 raise the elevator up higher than the pole
                    robot.grabber.cancelAutoAssist();
                    robot.grabber.close();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotDrive.driveBase.stop();
                    robot.elevator.setTarget(0.5, RobotParams.HIGH_JUNCTION_SCORING_HEIGHT, true, 1.0, event, 4.0);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;
                default:
                case DONE:
                    stop();
                    if (onFinishEvent != null)
                    {
                        onFinishEvent.signal();
                        onFinishEvent = null;
                    }
                    break;
            }

            robot.globalTracer.traceStateInfo(sm.toString(), sm.getState());
        }

    }   //autoAssistTask
    private void strafeRightToFindCone(Object context){
        strafeTheOtherWay = true;
    }
    private void grabberCancelDrive(Object context){
        robot.robotDrive.cancel();
    }
}   //class TaskPickupCone
