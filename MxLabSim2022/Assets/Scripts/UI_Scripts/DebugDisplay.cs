using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class DebugDisplay : MonoBehaviour
{
    readonly Dictionary<string, string> debugLogs = new Dictionary<string, string>();
    public Text display;
    public URController ur;
    public UGVControllerInput ugv;

    private void FixedUpdate()
    {
        if (SceneManager.GetActiveScene().name == "URScene")
            PrintURMessage();

        if (SceneManager.GetActiveScene().name == "UGVScene")
            PrintUGVMessage();
    }


    // Subcribe and Unsubcribe on gameobject enabled and disable
    private void OnEnable()
    {
        Application.logMessageReceived += HandleLog;
    }

    private void OnDisable()
    {
        Application.logMessageReceived -= HandleLog;
    }


    // Printing helper
    void HandleLog(string logString, string stackTrace, LogType type)
    {
        if (type == LogType.Log)
        {
            string[] splitString = logString.Split(char.Parse(":"));
            string debugKey = splitString[0];
            string debugValue = "";
            if (splitString.Length > 1)
                debugValue = splitString[1];

            if (debugLogs.ContainsKey(debugKey))
                debugLogs[debugKey] = debugValue;
            else
                debugLogs.Add(debugKey, debugValue);
        }

        string displayText = "";
        foreach (KeyValuePair<string, string> log in debugLogs)
        {
            if (log.Value == "")
                displayText += log.Key + "\n";
            else
                displayText += log.Key + ": " + log.Value + "\n";
        }
        display.text = displayText;
    }


    // Robot messages
    public void PrintURMessage()
    {
        float[] q = ur.GetJointAngles();
        Debug.Log("Joint: " + ur.GetJointName());
        Debug.Log("Q: " + q[0] + ", " + q[1] + ", " + q[2] + ", " + q[3] + ", " + q[4] + ", " + q[5]);
    }

    public void PrintUGVMessage()
    {
        Debug.Log("UGV Pos: " + ugv.GetPosition().x + "|" + ugv.GetPosition().z);
    }
}
