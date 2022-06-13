using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;

public class PlayerContinuousMovement : MonoBehaviour
{
    public GameObject xrRig;
    [SerializeField] private float speed = 10f;

    private XRNode node = XRNode.LeftHand;
    private readonly List<InputDevice> devices = new List<InputDevice>();
    private InputDevice device;
    // Start is called before the first frame update
    void GetDevice()
    {
        InputDevices.GetDevicesAtXRNode(node, devices);
        device = devices.FirstOrDefault();
    }

    private void OnEnable()
    {
        if (!device.isValid)
            GetDevice();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (!device.isValid)
            GetDevice();

        InputFeatureUsage<Vector2> joystickUsage = CommonUsages.primary2DAxis;

        if (device.TryGetFeatureValue(joystickUsage, out Vector2 joystickValue))
        {
            if (joystickValue.y > .5f)
                xrRig.transform.Translate(0, 0, speed * Time.fixedDeltaTime);

            if (joystickValue.y < -.5f)
                xrRig.transform.Translate(0, 0, -speed * Time.fixedDeltaTime);

            if (joystickValue.x > .5f)
                xrRig.transform.Translate(speed * Time.fixedDeltaTime, 0, 0);

            if (joystickValue.x < -.5f)
                xrRig.transform.Translate(-speed * Time.fixedDeltaTime, 0, 0);

        }
    }
}
