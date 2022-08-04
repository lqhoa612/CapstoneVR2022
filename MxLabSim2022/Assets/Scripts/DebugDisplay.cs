using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DebugDisplay : MonoBehaviour
{
    readonly Dictionary<string, string> debugLogs = new Dictionary<string, string>();
    public Text display;
    public URControllerInput _UR;
    public UGVControllerInput _UGV;
    public bool isUR = true;
    public URPosRot _urPosRot;

    private string _jointName = "none";

    private void FixedUpdate()
    {
        Debug.Log("Time: " + Time.time);
        if (isUR)
        {
            Debug.Log("Joint: " + _UR.GetControlledJoint());
            Debug.Log("Pos: " + _urPosRot.GetPosition());
            Debug.Log("Rot: " + _urPosRot.GetRotation());
            Debug.Log("Collision detected: " + _jointName);
        }
        if (!isUR)
            Debug.Log("UGV Pos: " + _UGV.GetPosition().x + "|" +_UGV.GetPosition().z);
    }

    private void OnEnable()
    {
        Application.logMessageReceived += HandleLog;
    }

    private void OnDisable()
    {
        Application.logMessageReceived -= HandleLog;
    }

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

    public void PrintURMessage(string jointName)
    {
        if (jointName != "")
            _jointName = "@ " + jointName;
        else
            _jointName = "none";
    }

}
