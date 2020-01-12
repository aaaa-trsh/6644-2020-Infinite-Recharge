/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Classes;

/**
 * Add your docs here.
 */
public class SerialDecoder 
{
    public static boolean isInteger(String string) 
    {
        try {
            Integer.valueOf(string);
            return true;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    public static int DecodeSerial(char varToFind, String input) {
        int retVal = -1;
        String rawinput = input;
        boolean found = false;
        int varIndex = 0;
        char endChar = ']';

        for (int index = 0; index < rawinput.length(); index++) {
            if (rawinput.toCharArray()[index] == varToFind && !found) {
                varIndex = index;
                found = true;
                break;
            }
        }

        if (found) {
            StringBuilder sb = new StringBuilder();

            for (int indexOffset = varIndex + 2; indexOffset < rawinput.length(); indexOffset++) {
                if (rawinput.toCharArray()[indexOffset] != endChar) {
                    sb.append(rawinput.toCharArray()[indexOffset]);
                } else {
                    String strVal = sb.toString();

                    if (isInteger(strVal)) {
                        retVal = Integer.parseInt(sb.toString());
                    }
                }
            }
        }
        return retVal;
    }
}
