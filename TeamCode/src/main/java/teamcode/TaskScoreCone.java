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
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;

/**
 * This class implements auto-assist task to score a cone.
 */
public class TaskScoreCone
{
    private static final String moduleName = "TaskScoreCone";

    public enum State
    {
        START,
        DONE
    }

    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject autoAssistTaskObj;
    private TrcEvent onFinishEvent;
    private double scoreHeight;
    private double expireTime;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskScoreCone(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        autoAssistTaskObj = TrcTaskMgr.createTask(moduleName, this::autoAssistTask);
    }   //TaskScoreCone

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
     * This method starts the auto-assist task to score a cone.
     *
     * @param scoreHeight specifies the elevator height to score the cone.
     * @param event specifies the event to signal when the auto-assist task is done, can be null if not provided.
     * @param timeout specifies the maximum time in seconds for the operation, can be zero if no timeout.
     */
    public void autoAssistScoreCone(double scoreHeight, TrcEvent event, double timeout)
    {
        this.scoreHeight = scoreHeight;
        this.onFinishEvent = event;
        this.expireTime = timeout > 0.0? TrcUtil.getCurrentTime() + timeout: 0.0;
        setTaskEnabled(true);
    }   //autoAssistScoreCone

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
        // 1. Raise elevator to scoring height.
        // 2. Call autoAssistFindPole.
        // 3. Set arm to calculated angle.
        // 4. Lower elevator to cap the pole.
        // 5. Release grabber.
        // 6. Raise arm and lower elevator.
        //
        if (state != null)
        {
            switch (state)
            {
                case START:
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

}   //class TaskScoreCone
