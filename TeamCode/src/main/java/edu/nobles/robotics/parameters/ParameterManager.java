package edu.nobles.robotics.parameters;

import com.acmerobotics.dashboard.config.Config;

@Config

public class ParameterManager {
    public static int highChamberExtendUpPos = 6550;
    public static int highChamberExtendDownPos = highChamberExtendUpPos *312/435;
    public static int highChamberRetractUpPos = 4150;
    public static int highChamberRetractDownPos = highChamberExtendDownPos *312/435;
    public static int wallUpPos = 150;

    public static double vertSlideUpStallCurrent = 3500;
    public static double vertSlideDownStallCurrent = vertSlideUpStallCurrent;

}
