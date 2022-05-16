using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.XR.Interaction.Toolkit;

public class ButtonEvent : XRBaseInteractable
{
    public UnityEvent OnPress, OnRelease;

    private float yMin, yMax;
    private bool wasPressed;

    private float initialHandHeight;
    private XRBaseInteractor interactor_;

    [System.Obsolete]
    protected new void Awake()
    {
        base.Awake();
        onHoverEntered.AddListener(StartPress);
        onHoverExited.AddListener(EndPress);
    }

    [System.Obsolete]
    private new void OnDestroy()
    {
        onHoverEntered.RemoveListener(StartPress);
        onHoverExited.RemoveListener(EndPress);
    }

    private void StartPress(XRBaseInteractor interactor)
    {
        interactor_ = interactor;
        initialHandHeight = GetLocalYPosition(interactor_.transform.position);
    }

    private void EndPress(XRBaseInteractor interactor)
    {
        interactor_ = null;
        initialHandHeight = .0f;

        wasPressed = false;
        SetYPosition(yMax);
    }

    private void Start()
    {
        SetLimit();
    }

    // Set the min and max value where button will trigger
    private void SetLimit()
    {
        Collider collider = GetComponent<Collider>();
        yMin = transform.localPosition.y - (collider.bounds.size.y * 1 / 3f);
        yMax = transform.localPosition.y;
    }

    public override void ProcessInteractable(XRInteractionUpdateOrder.UpdatePhase updatePhase)
    {
        if (interactor_)
        {
            float newHandHeight = GetLocalYPosition(interactor_.transform.position);
            float handDifference = initialHandHeight - newHandHeight;
            initialHandHeight = newHandHeight;

            float newPosition = transform.localPosition.y - handDifference;
            SetYPosition(newPosition);

            IsPressed();
        }
    }

    private float GetLocalYPosition(Vector3 position)
    {
        Vector3 localPosition = transform.root.InverseTransformPoint(position);
        return localPosition.y;
    }

    private void SetYPosition(float position)
    {
        Vector3 newPosition = transform.localPosition;
        newPosition.y = Mathf.Clamp(position, yMin, yMax);
        transform.localPosition = newPosition;
    }

    private void IsPressed()
    {
        bool inPosition = IsInPosition();
        if (inPosition && inPosition != wasPressed)
        {
            OnPress.Invoke();
        }
        else
        {
            OnRelease.Invoke();
        }
        wasPressed = inPosition;
    }

    private bool IsInPosition()
    {
        float acceptableRange = Mathf.Clamp(transform.localPosition.y, yMin, yMin + .01f);
        return transform.localPosition.y == acceptableRange;
    }
}
