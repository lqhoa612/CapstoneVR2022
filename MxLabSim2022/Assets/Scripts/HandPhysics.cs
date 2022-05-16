using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

public class HandPhysics : MonoBehaviour
{
    [SerializeField]  private float smoothingAmount = 15.0f;
    [SerializeField]  private Transform target;

    private Rigidbody rigidBody;
    private Vector3 targetPosition;
    private Quaternion targetRotation;

    private void Awake()
    {
        rigidBody = GetComponent<Rigidbody>();
    }

    private void Start()
    {
        TeleportToTarget(); // tele hands to controllers
    }

    private void Update()
    {
        SetTargetPosition();
        SetTargetRotation();
    }

    private void SetTargetPosition()
    {
        float time = smoothingAmount * Time.unscaledDeltaTime;
        targetPosition = Vector3.Lerp(targetPosition, target.position, time);
    }

    private void SetTargetRotation()
    {
        float time = smoothingAmount * Time.unscaledDeltaTime;
        targetRotation = Quaternion.Slerp(targetRotation, target.rotation, time);
    }

    private void FixedUpdate()
    {
        MoveToController();
        RotateToController();
    }

    private void MoveToController()
    {
        Vector3 positionDelta = targetPosition - transform.position;

        rigidBody.velocity = Vector3.zero;
        rigidBody.MovePosition(transform.position + positionDelta);
    }

    private void RotateToController()
    {
        rigidBody.angularVelocity = Vector3.zero;
        rigidBody.MoveRotation(targetRotation);
    }

    public void TeleportToTarget()
    {
        targetPosition = target.position;
        targetRotation = target.rotation;

        transform.position = targetPosition;
        transform.rotation = targetRotation;
    }
}