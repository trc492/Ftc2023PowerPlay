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
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;

class CmdAutoHigh implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoHigh";

    private enum State
    {
        START_DELAY,
        DRIVE_TO_HIGH,
        RAISE_ARM,
        SCORE_CONE,
        PREPARE_PICKUP,
        GRAB_CONE,
        PREPARE_SCORE,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int signalPos = 0;
    // Todo: CodeReview: why public? Nobody outside of this class will access it.
    public int cycleCount = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoHigh(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
        sm.start(State.START_DELAY);
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
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            boolean traceState = true;
            String msg;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    robot.setAutoStartPosition(autoChoices);
                    // Call vision at the beginning to figure out the position of the duck.
                    if (robot.vision != null)
                    {
                        signalPos = robot.vision.getLastSignal();
                        msg = "Signal found at position " + signalPos;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }

                    if (signalPos == 0)
                    {
                        //
                        // We still can't see the signal, set to default position.
                        //
                        signalPos = 2;
                        msg = "No signal found, default to position " + signalPos;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }
                    //
                    // Do start delay if any.
                    //
                    if (autoChoices.startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.DRIVE_TO_HIGH);
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_HIGH);
                        break;
                    }

                case DRIVE_TO_HIGH:
                    //drive to score position
                    //raise elevator to scoring height
                    //turn turret to score position
                    //wait for pure pursuit to finish
                    // Todo: add option to do center high poles
                    //Points are 6 inches from the high junction, on the line drawn from the the high junction to the
                    //corresponding cone stack, facing the cone stack
                    // Todo: CodeReview: please change all purePursuit points to use robot.getAutoTargetPoint.
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        if (autoChoices.startPos == FtcAuto.StartPos.LEFT)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-1.243, -0.061, -104.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(1.243, -0.061, 104.0));
                        }
                    }
                    else
                    {
                        if (autoChoices.startPos == FtcAuto.StartPos.LEFT)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(1.243, 0.061, 76.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-1.243, 0.061, -76.0));
                        }
                    }
                    robot.turret.setTarget(180.0);
                    sm.waitForSingleEvent(event, State.RAISE_ARM);
                    robot.elevator.setPresetPosition(2);
                    break;

                //raise arm to scoring position, wait for arm to finish
                case RAISE_ARM:
                    robot.arm.setPresetPosition(2, event, null);
                    sm.waitForSingleEvent(event, State.SCORE_CONE);
                    break;

                //dump the cone with auto-assist
                case SCORE_CONE:
                    //todo: write code for alignment to pole with vision
                    robot.intake.autoAssist(RobotParams.INTAKE_POWER_DUMP, event, null, 0.0);
                    sm.waitForSingleEvent(event, State.PREPARE_PICKUP);
                    break;

                //this is the first step in the cycle sequence(picking up the cone from the stack)
                //go park if we've already done 5 cycles or we have lest than 5 seconds left
                //turn turret to pickup position
                //set elevator to pickup height
                //set arm to pick up position
                //move robot toward cone stack
                //wait for pure pursuit
                case PREPARE_PICKUP:
                    if (TrcUtil.getModeElapsedTime() >= 27 || cycleCount == 5)
                    {
                        sm.setState(State.PARK);
                    }
                    else
                    {
                        robot.turret.setPresetPosition(0);
                        robot.elevator.setPresetPosition(2);
                        //todo: find arm pos
                        robot.arm.setPresetPosition(2);
                        //Todo: add code that moves robot FORWARDS x inches (but still in abs coordinates) to the cone pile
                        sm.waitForSingleEvent(event, State.GRAB_CONE);
                    }
                    break;

                case GRAB_CONE:
                    //todo: call auto assist pickup
                    robot.intake.autoAssist(RobotParams.INTAKE_POWER_PICKUP, event, null, 0.0);
                    //lower elevator until pickup signal
                    robot.elevator.setPresetPosition(0);
                    sm.waitForSingleEvent(event, State.PREPARE_SCORE);
                    break;
                //prepare for scoring
                case PREPARE_SCORE:
                    //increment cycles by 1(5 is when we finished the cone stack)
                    robot.elevator.cancel();
                    cycleCount++;
                    robot.arm.setPresetPosition(2, event, null);
                    robot.turret.setPresetPosition(180);
                    robot.elevator.setPresetPosition(2);
                    //Todo: add code that moves robot BACKWARDS x inches (but still in abs coordinates) to the high junction
                    sm.waitForSingleEvent(event, State.SCORE_CONE);
                    break;

                case PARK:
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        // We are not parking anywhere, just stop and be done.
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        if (autoChoices.parking == FtcAuto.Parking.FAR_TILE)
                        {
                            //Todo: not sure how to use the constants in RobotParams

                        }
                        else
                        {

                        }
                    }

                    break;

                case DONE:
                default:
                    //
                    // We are done, zero calibrate the arm will lower it.
                    //
                    robot.arm.zeroCalibrate();
                    cancel();
                    break;
            }

            if (traceState)
            {
                robot.globalTracer.traceStateInfo(
                        sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                        robot.robotDrive.purePursuitDrive, null);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoHigh
