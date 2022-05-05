using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;


[RequireComponent(typeof(Animator))]
public class Hand :  MonoBehaviour
{
    //Animation
    Animator animator;
    private float gripTarget, gripCurrent;
    private float triggerTarget, triggerCurrent;
    [SerializeField] private float AnimationSpeed = 10;

    //Physics Movement
    [SerializeField] private GameObject followObj;
    [SerializeField] private float followSpeed = 30f;
    [SerializeField] private float rotateSpeed = 30f;
    [SerializeField] private Vector3 positionOffsetValue;
    [SerializeField] private Vector3 rotationOffsetValue;
    private Transform followTarget;
    private Rigidbody body;


    private void Start()
    {
        //Animation
        animator = GetComponent<Animator>();

        //Physics
        followTarget = followObj.transform;
        body = GetComponent<Rigidbody>();
        body.collisionDetectionMode = CollisionDetectionMode.Continuous;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        //body.mass = 20f;

        //Get hands
        body.position = followTarget.position;
        body.rotation = followTarget.rotation;
    }

    private void Update()
    {
        AnimateHand();
        PhysicsMove();
    }

    private void PhysicsMove()
    {
        //Position
        var positionOffset = followTarget.position + positionOffsetValue;
        var dist = Vector3.Distance(positionOffset, transform.position);
        body.velocity = (positionOffset - transform.position).normalized * (followSpeed * dist);

        //Rotation
        var rotationOffset = followTarget.rotation * Quaternion.Euler(rotationOffsetValue);
        var quat = rotationOffset * Quaternion.Inverse(body.rotation);
        quat.ToAngleAxis(out float angle, out Vector3 axis);
        body.angularVelocity = axis * (angle * Mathf.Deg2Rad * rotateSpeed);
    }

    internal void setGrip(float v) { gripTarget = v; }

    internal void setTrigger(float v) { triggerTarget = v; }

    void AnimateHand()
    {
        if (gripCurrent != gripTarget)
        {
            gripCurrent = Mathf.MoveTowards(gripCurrent, gripTarget, Time.deltaTime* AnimationSpeed);
            animator.SetFloat("Grip", gripCurrent);
        }
        if (triggerCurrent != triggerTarget)
        {
            triggerCurrent = Mathf.MoveTowards(triggerCurrent, triggerTarget, Time.deltaTime* AnimationSpeed);
            animator.SetFloat("Trigger", triggerCurrent);
        }
    }

}
