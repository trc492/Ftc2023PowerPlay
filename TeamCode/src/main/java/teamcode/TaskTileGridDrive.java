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

import java.util.ArrayList;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;

/**
 * This class implements the auto-assist tile grid drive. It allows the driver to use the DPad to quickly navigate
 * the field maze in square pattern accurately without the risk of running into the junction poles. This algorithm
 * assumes we have accurate odometry. If we don't, all bets are off.
 */
public class TaskTileGridDrive
{
    private static final String moduleName = "TaskTileGridDrive";

    private enum State
    {
        START,
        DONE
    }   //enum State

    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject tileGridDriveTaskObj;
    private final ArrayList<TrcPose2D> gridDriveQueue = new ArrayList<>();
    private TrcDbgTrace msgTracer = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public TaskTileGridDrive(Robot robot)
    {
        this.robot = robot;
        this.event = new TrcEvent(moduleName);
        this.sm = new TrcStateMachine<>(moduleName);
        tileGridDriveTaskObj = TrcTaskMgr.createTask("tileGridDriveTask", this::tileGridDriveTask);
    }   //TaskTileGridDrive

    /**
     * This method enables/disables tracing for the auto-assist task.
     *
     * @param tracer specifies the tracer to use for logging events.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        msgTracer = tracer;
    }   //setMsgTracer

    /**
     * This method cancels the tileGridDrive task.
     */
    public void cancel()
    {
        setTaskEnabled(false);
        gridDriveQueue.clear();
    }   //cancel

    /**
     * This method enables/disables the tileGridDrive task.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        if (enabled && !sm.isEnabled())
        {
            sm.start(State.START);
            tileGridDriveTaskObj.registerTask(TrcTaskMgr.TaskType.SLOW_POSTPERIODIC_TASK);
        }
        else if (!enabled && sm.isEnabled())
        {
            sm.stop();
            tileGridDriveTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method checks if the task is enabled.
     *
     * @return true if the tileGridDrive task is enabled, false otherwise.
     */
    public boolean isTaskEnabled()
    {
        return sm.isEnabled();
    }   //isTaskEnabled

    /**
     * This method adds a X movement segment to the array list.
     *
     * @param tiles specifies the X movement in the unit of tiles.
     */
    public void setRelativeXTileTarget(int tiles)
    {
        TrcPose2D lastNode = !gridDriveQueue.isEmpty()? gridDriveQueue.get(gridDriveQueue.size() - 1): null;

        if (lastNode != null)
        {
            lastNode.x += tiles;
            if (lastNode.x == 0.0)
            {
                gridDriveQueue.remove(lastNode);
            }
        }
        else
        {
            lastNode = new TrcPose2D(tiles, 0.0, Math.signum(tiles) * 90.0);
            gridDriveQueue.add(lastNode);
        }

        setTaskEnabled(true);
    }   //setRelativeXTileTarget

    /**
     * This method adds a Y movement segment to the array list.
     *
     * @param tiles specifies the Y movement in the unit of tiles.
     */
    public void setRelativeYTileTarget(int tiles)
    {
        TrcPose2D lastNode = !gridDriveQueue.isEmpty()? gridDriveQueue.get(gridDriveQueue.size() - 1): null;

        if (lastNode != null)
        {
            lastNode.y += tiles;
            if (lastNode.y == 0.0)
            {
                gridDriveQueue.remove(lastNode);
            }
        }
        else
        {
            // If the robot is moving in the positive y direction, heading should be north (0)--otherwise, heading
            // should be south (180).
            lastNode = new TrcPose2D(0.0, tiles, (-Math.signum(tiles) + 1) * 90.0);
            gridDriveQueue.add(lastNode);
        }

        setTaskEnabled(true);
    }   //setRelativeYTileTarget

    /**
     * This periodic task navigates the robot through the field maze according to the array list of movement segments.
     *
     * @param taskType specifies the task type (not used).
     * @param runMode specifies the robot run mode (not used).
     */
    private void tileGridDriveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            boolean traceState = true;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START:
                    TrcPose2D targetTilePose = !gridDriveQueue.isEmpty()? gridDriveQueue.remove(0): null;
                    if (targetTilePose != null)
                    {
                        TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                        robot.robotDrive.purePursuitDrive.start(
                            event, robotPose, false,
                            new TrcPose2D(
                                (tileCenterPosition(robotPose.x) + targetTilePose.x) * RobotParams.FULL_TILE_INCHES,
                                (tileCenterPosition(robotPose.y) + targetTilePose.y) * RobotParams.FULL_TILE_INCHES,
                                targetTilePose.angle));
                        sm.waitForSingleEvent(event, State.START);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                default:
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
    }   //tileGridDriveTask

    /**
     * This method rounds the given position value to the center of a full tile, meaning it will round the value so
     * that the position will be at the center of the tile in tile unit.
     *
     * @param position specifies the field position in inches.
     * @return field position at the center of the tile in tile unit.
     */
    private double tileCenterPosition(double position)
    {
        return ((int) (position/RobotParams.FULL_TILE_INCHES)) + 0.5;
    }   //tileCenterPosition

}   //class TaskTileGridDrive
