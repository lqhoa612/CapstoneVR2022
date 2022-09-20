using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    public URController control;
    public JointStatePub publisher;

    private void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("robot")) return;
        publisher.safeToPublish = false;
        control.mode = URController.ControlMode.Stopped;
        Debug.LogWarning("Collision Detected.");
    }

    private void OnTriggerExit(Collider other)
    {
        if (!other.CompareTag("robot")) return;
        publisher.safeToPublish = true;
        Debug.LogWarning("No Collision.");
    }
}
