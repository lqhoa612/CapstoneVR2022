using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class DebugDisplay : MonoBehaviour
{
    readonly Dictionary<string, string> debugLogs = new Dictionary<string, string>();
    public Text display;
    public URController ur;
    public GameObject target;
    public UGVControllerInput ugv;

    private void FixedUpdate()
    {
        if (ur != null)
        {
            if (SceneManager.GetActiveScene().name == "URScene")
                PrintURMessage();

            if (SceneManager.GetActiveScene().name == "URAutoScene")
                PrintURAutoMessage();
        }
        
        if (SceneManager.GetActiveScene().name == "UGVScene" && ugv != null)
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


    // Print helper
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
        Debug.Log("Joint: " + ur.selectedJoint);
        Debug.Log("Q: " + Mathf.Round(q[0]) + ", " + Mathf.Round(q[1]) + ", " + Mathf.Round(q[2]) + ", " + Mathf.Round(q[3]) + ", " + Mathf.Round(q[4]) + ", " + Mathf.Round(q[5]));

        if (ur.collisionMsg != null)
        {
            Debug.Log("\nCollision check: " + ur.collisionMsg);
        }
        else
        {
            Debug.Log("\nCollision check: ");
        }

        // Control instruction
        //if (ur.mode == URController.ControlMode.Auto)
        //{
        //    Debug.Log("Right trigger: move EE to target position");
        //    Debug.Log("Target: " + ur.service.target.transform.localPosition);
        //}
        //Debug.Log("Right joystick: rotate joint");
        //Debug.Log("B: next joint | A: previous joint");
        //Debug.Log("Left Menu: call menu");
        //Debug.Log("Left trigger: select mode/scene");
    }

    public void PrintURAutoMessage()
    {
        float[] q = ur.GetJointAngles();
        Debug.Log("Q: " + Mathf.Round(q[0]) + ", " + Mathf.Round(q[1]) + ", " + Mathf.Round(q[2]) + ", " + Mathf.Round(q[3]) + ", " + Mathf.Round(q[4]) + ", " + Mathf.Round(q[5]));
        Debug.Log("Target: " + target.transform.localPosition);

        if (ur.collisionMsg != null)
        {
            Debug.Log("\nCollision check: " + ur.collisionMsg);
        }
        else
        {
            Debug.Log("\nCollision check: ");
        }

    }

    public void PrintUGVMessage()
    {
        Debug.Log("UGV Pos: " + ugv.GetPosition().x + "|" + ugv.GetPosition().z);
    }
}
