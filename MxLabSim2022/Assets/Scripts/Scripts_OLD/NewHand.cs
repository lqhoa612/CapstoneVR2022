using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Animator))]
public class NewHand : MonoBehaviour
{
    Animator animator;
    private float gripParam_;//, triggerParam_;
    private float gripParam_target;//, triggerParam_target;
    [SerializeField] private float AnimationSpeed = 10;

    private void Start()
    {
        animator = GetComponent<Animator>();
    }

    private void Update()
    {
        AnimateHand();
    }

    internal void SetGrip(float gripParam) => gripParam_ = gripParam;
    //internal void setTrigger(float triggerParam) => triggerParam_ = triggerParam;

    private void AnimateHand()
    {
        if (gripParam_ != gripParam_target)
        {
            gripParam_ = Mathf.MoveTowards(gripParam_, gripParam_target, Time.deltaTime * AnimationSpeed);
            animator.SetFloat("Grip", gripParam_);
        }

        /*if (triggerParam_ != triggerParam_target)
        {
            triggerParam_ = Mathf.MoveTowards(triggerParam_, triggerParam_target, Time.deltaTime * AnimationSpeed);
            animator.SetFloat("Trigger", triggerParam_);
        }*/
    }

}
