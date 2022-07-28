﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

public class ArticulationJointController : MonoBehaviour
{
    public RotationDirection rotationState = RotationDirection.None;
    private float speed = 50.0f;
    private ArticulationBody articulation;

    public List<string> tags;


    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
    }

    void FixedUpdate() 
    {
        if (rotationState != RotationDirection.None) {
            float rotationChange = (float)rotationState * speed * Time.fixedDeltaTime;
            float rotationGoal = CurrentPrimaryAxisRotation() + rotationChange;
            RotateTo(rotationGoal);
        }
    }

    private void OnCollisionEnter(UnityEngine.Collision collision)
    {
        foreach (var item in tags)
        {
            if (collision.collider.CompareTag(item))
            {
                Debug.LogWarning("Bruh!");
                speed = 0f;
            }
        }
    }

    private void OnCollisionExit(UnityEngine.Collision collision)
    {
        foreach (var item in tags)
        {
            if (collision.collider.CompareTag(item))
            {
                speed = 50.0f;
            }
        }
    }


    // MOVEMENT HELPERS

    float CurrentPrimaryAxisRotation()
    {
        float currentRotationRads = articulation.jointPosition[0];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    void RotateTo(float primaryAxisRotation)
    {
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }

}
