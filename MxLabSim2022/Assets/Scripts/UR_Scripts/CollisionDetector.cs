using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    public URController control;
    URController.ControlMode prev_mode;

    private void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("robot")) return;
        prev_mode = control.mode;
        control.mode = URController.ControlMode.Stopped;
        Debug.LogWarning("Collision Detected.");
    }

    private void OnTriggerExit(Collider other)
    {
        if (!other.CompareTag("robot")) return;
        Debug.LogWarning("No Collision.");
    }
}
