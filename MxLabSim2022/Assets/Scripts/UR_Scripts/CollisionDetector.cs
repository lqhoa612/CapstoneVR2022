using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    public URController control;

    private void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("robot")) return;

        control.StopAll();
        Debug.LogWarning("Hit");
    }

    private void OnTriggerExit(Collider other)
    {
        if (!other.CompareTag("robot")) return;
        control.speed = 20;
    }
}
