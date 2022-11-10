/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

class CmdVisionTest implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoHigh";

    private enum State
    {
        DRIVE_TO_CONE_STACK,
        TURN_TO_CONES,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int signalPos = 0;
    public int cycleCount = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdVisionTest(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);

        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
        sm.start(State.DRIVE_TO_CONE_STACK);
    }   //CmdAutoFarCarousel

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        robot.robotDrive.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime) {
        State state = sm.checkReadyAndGetState();

        if (state == null) {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        } else {
            boolean traceState = true;
            String msg;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state) {
                case DRIVE_TO_CONE_STACK:
                    //
                    robot.robotDrive.driveBase.setFieldPosition(RobotParams.STARTPOS_RED_LEFT);
                    // Disable TensorFlow if we are not using it to improve PurePursuitDrive performance.
                    robot.robotDrive.purePursuitDrive.start(
                            event, null, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.5, -0.5, 0.0));//,
                            //robot.robotDrive.pathPoint(0, 0, -90));                    //
                    //
                    // Set robot starting position in the field.
                    //
                    sm.waitForSingleEvent(event, State.DONE);//TURN_TO_CONES);
                    break;

                case TURN_TO_CONES:
                    double angle = 0;
                    if(robot.vision.frontEocvVision != null){
                        TrcVisionTargetInfo<?> target = robot.vision.getBestDetectedTargetInfo(null);
                        if(target != null){
                            angle = target.horizontalAngle;
                        }
                    }
                    robot.robotDrive.purePursuitDrive.start(
                            event, null, robot.robotDrive.driveBase.getFieldPosition(), true,
                            robot.robotDrive.pathPoint(0, 0, angle));
                    break;

                case DONE:
                default:
                    //
                    // We are done, zero calibrate the arm will lower it.
                    //
                    cancel();
                    break;
            }

            if (traceState) {
                robot.globalTracer.traceStateInfo(
                        sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                        robot.robotDrive.purePursuitDrive,
                        null);
            }
        }

        return !sm.isEnabled();
    }
}   //cmdPeriodic

