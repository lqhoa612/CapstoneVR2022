using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewHandPhysic : MonoBehaviour
{
    public Transform handObject;
    private Rigidbody rb;
    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        rb.velocity = (handObject.position - transform.position) / Time.fixedDeltaTime;

        Quaternion rotationDiff = handObject.rotation * Quaternion.Inverse(transform.rotation);
        rotationDiff.ToAngleAxis(out float angleInDegree, out Vector3 rotationAxis);
        Vector3 rotationDiffInDegree = angleInDegree * rotationAxis;
        rb.angularVelocity = rotationDiffInDegree * Mathf.Deg2Rad / Time.fixedDeltaTime;
    }
}
