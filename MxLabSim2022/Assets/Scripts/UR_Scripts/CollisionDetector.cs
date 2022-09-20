using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    public URController control;
    public JointStatePub publisher;
    //URController.ControlMode prev_mode;

    private void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("robot")) return;
        publisher.safeToPublish = false;
        //prev_mode = control.mode;
        control.mode = URController.ControlMode.Stopped;
        Debug.LogWarning("Collision Detected.");
    }

    private void OnTriggerExit(Collider other)
    {
        if (!other.CompareTag("robot")) return;
        //control.mode = prev_mode;
        publisher.safeToPublish = true;
        Debug.LogWarning("No Collision.");
    }
}
