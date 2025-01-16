package org.firstinspires.ftc.teamcode;
import java.util.*;

public class CommandSystem
{
    // stores a state correlating to a subsystem
    Map<char,boolean> completedMap = new Hashtable<char,boolean>();
    // ArrayList<String> completedList = new ArrayList<String>();
    
    public boolean GetBoolsCompleted()
    {
        // Enumeration<boolean> values = completedDict.elements();
        // boolean currentValue = true;

        // for every key in the dictionary
        for (boolean value : completedMap.values())
        {
            if (!value)
            {
                return false;
            }
        }
        return true;

        // obsolete ? 
        // while (values.hasMoreElements())
        // {
        //     currentValue = values.nextElement();
        //     if (!currentValue)
        //     {
        //         return false;
        //     }
        // }
        // return true;
    }
    
    // clear entire map for next command
    public void ResetMap()
    {
        completedMap.Clear();
    }       
    
    // set a given element's completion state to false
    public void SetElementFalse(char element)
    {
        completedMap.put(element,false);
    }

    // set a given element's completion state to true
    public void SetElementTrue(char element)
    {
        completedMap.put(element,true);
    }

    // gets the state of an element
    public boolean GetElementState(char element)
    {
        return completedMap.get(element);
    }
    
    // returns the map itself
    public Map GetMap()
    {
        return completedMap;
    }
}