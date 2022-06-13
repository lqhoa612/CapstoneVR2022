using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;

public class PlayerContinuousTurn : MonoBehaviour
{
    public GameObject xrRig;
    [SerializeField] private float speed = 10f;

    private XRNode node = XRNode.RightHand;
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
            if (joystickValue.x > .5f)
                xrRig.transform.Rotate(0, speed * Time.fixedDeltaTime, 0, Space.Self);

            if (joystickValue.x < -.5f)
                xrRig.transform.Rotate(0, -speed * Time.fixedDeltaTime, 0, Space.Self);

        }
    }
}
