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

package multiteams.team6541;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcGamepad;
import multiteams.FtcTest;

/**
 * This class contains the Test Mode program.
 */
public class FtcTest6541 extends FtcTeleOp6541
{
    private final FtcTest ftcTest = new FtcTest();

    //
    // Overrides FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        //
        // TeleOp initialization.
        //
        super.initRobot();
        //
        // Common Test initialization.
        //
        ftcTest.init(robot);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called before test mode is about to start so it can initialize appropriate subsystems for the
     * test.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        super.startMode(prevMode, nextMode);
        ftcTest.startMode(prevMode, nextMode);
    }   //startMode

    /**
     * This method is called before test mode is about to exit so it can do appropriate cleanup.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        ftcTest.stopMode(prevMode, nextMode);
        super.stopMode(prevMode, nextMode);
    }   //stopMode

    /**
     * This method is called periodically at a fast rate. Typically, you put code that requires servicing at a
     * high frequency here. To make the robot as responsive and as accurate as possible especially in autonomous
     * mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void fastPeriodic(double elapsedTime)
    {
        ftcTest.fastPeriodic(elapsedTime);
    }   //fastPeriodic

    /**
     * This method is called periodically at a slow rate. Typically, you put code that doesn't require frequent
     * update here. For example, TeleOp joystick code or status display code can be put here since human responses
     * are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void slowPeriodic(double elapsedTime)
    {
        if (ftcTest.allowTeleOp())
        {
            //
            // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
            //
            super.slowPeriodic(elapsedTime);
        }

        ftcTest.slowPeriodic(elapsedTime);
    }   //slowPeriodic

    //
    // Overrides TrcGameController.ButtonHandler in TeleOp.
    //

    /**
     * This method is called when a driver gamepad button event occurs.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (ftcTest.allowTeleOp())
        {
            //
            // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
            // FtcTeleOp gamepad actions.
            //
            robot.dashboard.displayPrintf(7, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");

            boolean processed = false;
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }
            //
            // If the control was not processed by this method, pass it back to TeleOp.
            //
            if (!processed)
            {
                super.driverButtonEvent(gamepad, button, pressed);
            }
        }
    }   //driverButtonEvent

    /**
     * This method is called when an operator gamepad button event occurs.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void operatorButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (ftcTest.allowTeleOp())
        {
            //
            // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
            // FtcTeleOp gamepad actions.
            //
            robot.dashboard.displayPrintf(7, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");

            boolean processed = false;
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }
            //
            // If the control was not processed by this method, pass it back to TeleOp.
            //
            if (!processed)
            {
                super.operatorButtonEvent(gamepad, button, pressed);
            }
        }
    }   //operatorButtonEvent

}   //class FtcTest
