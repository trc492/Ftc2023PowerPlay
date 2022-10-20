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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcNotifier;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;

/**
 * Robot starts in between cone stack and high goal
 * starts on the middle of the line
 * facing the cone stack
 * score one cone and end where it started
 * turret starts facing forward
 * elevator down
 * arm position doesn't matter
 */
public class TaskCyclingCones
{
    private static final String moduleName = "TaskCyclingCones";

    public enum CycleType
    {
        FULL_CYCLE, //does a full cycle(pickup and scoring), used for auto and maybe teleop
        SCORING_ONLY,//assumes robot already has a cone, scores it onto the pole
        PICKUP_ONLY//picks up a cone from the stack
    }

    public enum VisionType
    {
        NO_VISION,
        FULL_VISION,//uses vision to horizontally align to target and determine distance to target
        ALIGN_VISION_ONLY//only uses vision to horizontally align to target
    }

    /*
     * 1. drive to the cone stack and raise elevator and set arm to 90 and spin intake wheels
     * 2. lower the elevator to the correct height
     * 3. raise the elevator up higher than the pole
     * 4. turn turret 180 degrees while driving backwards to the pole
     * 5. set arm to 90
     * 6. spin intake backwards
     * 7. turn turret 180 degrees and lower elevator while driving back to start position
     */
    public enum State
    {
        //Pickup states
        ALIGN_TO_CONE,        //horizontally align to the cone
        FIND_DIST_TO_CONE,    //Determine distance to cone(if doing FULL_VISION)
        DRIVE_TO_CONE,        //Drive forward if we determined distance to cone
        PREPARE_PICKUP,
        GRAB_CONE,
        TURN_AROUND,//if doing FULL_CYCLE, may need to rotate turret to switch from picking up cones to scoring them
        //Scoring states
        ALIGN_TO_POLE,
        FIND_DIST_TO_POLE,
        DRIVE_TO_POLE,
        PREPARE_SCORE,
        SCORE_CONE,
        DONE
    }

    private final Robot robot;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject cycleTaskObj;
    private TrcDbgTrace msgTracer = null;
    private CycleType cycleType = CycleType.FULL_CYCLE;
    private VisionType visionType = VisionType.FULL_VISION;
    private TrcEvent onFinishEvent = null;
    private TrcNotifier.Receiver onFinishCallback = null;

    public TaskCyclingCones(Robot robot)
    {
        this.robot = robot;
        sm = new TrcStateMachine<>(moduleName);
        cycleTaskObj = TrcTaskMgr.createTask(moduleName, this::cycleTask);
    }   //TaskCyclingCones

    /**
     * This method enables/disables tracing for the auto-assist task.
     *
     * @param tracer specifies the tracer to use for logging events.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        msgTracer = tracer;
    }   //setMsgTracer

    public void cancel()
    {
        sm.stop();
        cycleTaskObj.unregisterTask();

        if (onFinishEvent != null)
        {
            onFinishEvent.cancel();
            onFinishEvent = null;
        }

        if (onFinishCallback != null)
        {
            onFinishCallback.notify(null);
            onFinishCallback = null;
        }
    }   //cancel

    //prepare for cycling, start sm
    public void startCycling(CycleType cycleType, VisionType visionType)
    {
        this.cycleType = cycleType;
        this.visionType = visionType;
        cycleTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
        sm.start(cycleType == CycleType.SCORING_ONLY? State.ALIGN_TO_POLE: State.ALIGN_TO_CONE);
    }   //startCycling

    private void cycleTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            boolean traceState = true;
            double matchTime = TrcUtil.getModeElapsedTime();

            switch (state)
            {
                //Todo for 10/19: write out each of the steps in enum "State" in a similar format to CmdAutoHigh,
                //ignore vision steps(like aligning or determining distance for now
                //Pickup states
                //horizontally align to the cone
                case ALIGN_TO_CONE:
                    break;

                //Determine distance to cone(if doing FULL_VISION)
                case FIND_DIST_TO_CONE:
                    break;

                //Drive forward if we determined distance to cone
                case DRIVE_TO_CONE:
                    break;

                case PREPARE_PICKUP:
                    break;

                case GRAB_CONE:
                    break;

                //if doing FULL_CYCLE, may need to rotate turret to switch from picking up cones to scoring them
                case TURN_AROUND:
                    break;

                //Scoring states
                case ALIGN_TO_POLE:
                    break;

                case FIND_DIST_TO_POLE:
                    break;

                case DRIVE_TO_POLE:
                    break;

                case PREPARE_SCORE:
                    break;

                case SCORE_CONE:
                    break;

                default:
                case DONE:
                    cancel();
                    break;
            }

            if (msgTracer != null && traceState)
            {
                msgTracer.traceStateInfo(
                    sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
            }
        }
    }   //cycleTask

}   //class TaskCyclingCones
