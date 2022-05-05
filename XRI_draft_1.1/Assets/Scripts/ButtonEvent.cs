using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.XR.Interaction.Toolkit;

public class ButtonEvent : XRBaseInteractable
{
    public UnityEvent OnPress;

    private float yMin, yMax;
    private bool wasPressed;

    private float initialHandHeight;
    private XRBaseInteractor interactor_;

    [System.Obsolete]
    protected new void Awake()
    {
        base.Awake();
        onHoverEnter.AddListener(StartPress);
        onHoverExit.AddListener(EndPress);
    }

    [System.Obsolete]
    private new void OnDestroy()
    {
        onHoverEnter.RemoveListener(StartPress);
        onHoverExit.RemoveListener(EndPress);
    }

    private void StartPress(XRBaseInteractor interactor)
    {
        interactor_ = interactor;
        initialHandHeight = getLocalYPosition(interactor_.transform.position);
    }

    private void EndPress(XRBaseInteractor interactor)
    {
        interactor_ = null;
        initialHandHeight = .0f;

        wasPressed = false;
        setYPosition(yMax);
    }

    private void Start()
    {
        SetLimit();
    }

    // Set the min and max value where button will trigger
    private void SetLimit()
    {
        Collider collider = GetComponent<Collider>();
        yMin = transform.localPosition.y - (collider.bounds.size.y * 2 / 3f);
        yMax = transform.localPosition.y;
    }

    public override void ProcessInteractable(XRInteractionUpdateOrder.UpdatePhase updatePhase)
    {
        if (interactor_)
        {
            float newHandHeight = getLocalYPosition(interactor_.transform.position);
            float handDifference = initialHandHeight - newHandHeight;
            initialHandHeight = newHandHeight;

            float newPosition = transform.localPosition.y - handDifference;
            setYPosition(newPosition);

            isPressed();
        }
    }

    private float getLocalYPosition(Vector3 position)
    {
        Vector3 localPosition = transform.root.InverseTransformPoint(position);
        return localPosition.y;
    }

    private void setYPosition(float position)
    {
        Vector3 newPosition = transform.localPosition;
        newPosition.y = Mathf.Clamp(position, yMin, yMax);
        transform.localPosition = newPosition;
    }

    private void isPressed()
    {
        bool inPosition = isInPosition();
        if (inPosition && inPosition != wasPressed)
        {
            OnPress.Invoke();
        }
        wasPressed = inPosition;
    }

    private bool isInPosition()
    {
        float acceptableRange = Mathf.Clamp(transform.localPosition.y, yMin, yMin + .01f);
        return transform.localPosition.y == acceptableRange;
    }
}
