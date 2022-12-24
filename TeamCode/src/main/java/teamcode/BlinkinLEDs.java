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

import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcFtcLib.ftclib.FtcRevBlinkin;

/**
 * This class encapsulates the REV Blinkin LED controller to provide a priority indicator showing the status of the
 * robot.
 */
class BlinkinLEDs extends FtcRevBlinkin
{
    // LED pattern names.
    public static final String LABEL_BOLT = "1 Bolt";
    public static final String LABEL_BULB = "2 Bulb";
    public static final String LABEL_PANEL = "3 Panel";
    public static final String GOT_RED_CONE = "GotRedCone";
    public static final String GOT_BLUE_CONE = "GotBlueCone";
    public static final String GOT_YELLOW_POLE = "GotYellowPole";
    public static final String AUTOASSIST_GRABBER_ON = "AutoAssistGrabberOn";
    public static final String IMAGE1_NAME = "Red Audience Wall";
    public static final String IMAGE2_NAME = "Red Rear Wall";
    public static final String IMAGE3_NAME = "Blue Audience Wall";
    public static final String IMAGE4_NAME = "Blue Rear Wall";
    public static final String DRIVE_ORIENTATION_FIELD = "FieldMode";
    public static final String DRIVE_ORIENTATION_ROBOT = "RobotMode";
    public static final String DRIVE_ORIENTATION_INVERTED = "InvertedMode";

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name. This is also the REV Blinkin's hardware name.
     */
    public BlinkinLEDs(String instanceName)
    {
        super(instanceName);
        // LED Patterns are sorted in decreasing priority order.
        final TrcRevBlinkin.Pattern[] ledPatternPriorities = {
            // Highest priority.
            new TrcRevBlinkin.Pattern(LABEL_BOLT, TrcRevBlinkin.RevLedPattern.SolidRed),
            new TrcRevBlinkin.Pattern(LABEL_BULB, TrcRevBlinkin.RevLedPattern.SolidGreen),
            new TrcRevBlinkin.Pattern(LABEL_PANEL, TrcRevBlinkin.RevLedPattern.SolidBlue),
            new TrcRevBlinkin.Pattern(GOT_RED_CONE, TrcRevBlinkin.RevLedPattern.FixedStrobeRed),
            new TrcRevBlinkin.Pattern(GOT_BLUE_CONE, TrcRevBlinkin.RevLedPattern.FixedStrobeBlue),
            new TrcRevBlinkin.Pattern(GOT_YELLOW_POLE, RevLedPattern.FixedStrobeGold),
            new TrcRevBlinkin.Pattern(AUTOASSIST_GRABBER_ON, TrcRevBlinkin.RevLedPattern.FixedStrobeWhite),
            new TrcRevBlinkin.Pattern(IMAGE1_NAME, RevLedPattern.FixedLightChaseRed),
            new TrcRevBlinkin.Pattern(IMAGE2_NAME, RevLedPattern.FixedLightChaseBlue),
            new TrcRevBlinkin.Pattern(IMAGE3_NAME, RevLedPattern.FixedBreathRed),
            new TrcRevBlinkin.Pattern(IMAGE4_NAME, RevLedPattern.FixedBreathBlue),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_FIELD, TrcRevBlinkin.RevLedPattern.SolidViolet),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_ROBOT, TrcRevBlinkin.RevLedPattern.SolidWhite),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_INVERTED, TrcRevBlinkin.RevLedPattern.SolidGray)
            // Lowest priority.
        };
        setPatternPriorities(ledPatternPriorities);
    }   //BlinkinLEDs

}   //class BlinkinLEDs
