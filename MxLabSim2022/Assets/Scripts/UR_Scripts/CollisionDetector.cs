using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    public URController control;

    private void OnCollisionEnter(Collision col)
    {
        control.StopAll();
        Debug.Log("Hit");
    }
}
