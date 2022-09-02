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

package teamcode.team6541;

import TrcCommonLib.trclib.TrcRobot;
import teamcode.MecanumDrive;
import teamcode.Robot;
import teamcode.RobotParams;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot6541 extends Robot
{
    @SuppressWarnings("unused")
    RobotParams6541 robotParams = new RobotParams6541();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *                specific sensors and subsystems if necessary.
     */
    public Robot6541(TrcRobot.RunMode runMode)
    {
        super(runMode);
        //
        // If noRobot is true, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (!RobotParams.Preferences.noRobot)
        {
            //
            // Create and initialize RobotDrive.
            //
            robotDrive = new MecanumDrive(this);
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.initSubsystems)
            {
            }
        }

        speak("Init complete");
    }   //Robot6541

}   //class Robot6541
