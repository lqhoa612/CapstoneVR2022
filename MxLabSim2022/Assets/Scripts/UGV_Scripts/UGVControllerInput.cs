using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;

public class UGVControllerInput : MonoBehaviour
{
    public GameObject robot;
    [SerializeField] private float speed = 10.0f;

    private readonly XRNode ugvControlNode = XRNode.RightHand;
    private readonly List<InputDevice> ugv_devices = new List<InputDevice>();
    private InputDevice ugv_device;
    // Start is called before the first frame update
    void GetDevice()
    {
        InputDevices.GetDevicesAtXRNode(ugvControlNode, ugv_devices);
        ugv_device = ugv_devices.FirstOrDefault();
    }

    void OnEnable()
    {
        if (!ugv_device.isValid)
            GetDevice();
    }

    // Update is called once per frame
    private void FixedUpdate()
    {
        if (!ugv_device.isValid)
            GetDevice();

        InputFeatureUsage<Vector2> joystickUsage = CommonUsages.primary2DAxis;

        if (ugv_device.TryGetFeatureValue(joystickUsage, out Vector2 joystickValue))
        {
            if (joystickValue.y > .5f) 
                robot.transform.Translate(-speed/50 * Time.fixedDeltaTime, 0.0f, 0.0f);
                
            if (joystickValue.y < -.5f)
                robot.transform.Translate(speed/50 * Time.fixedDeltaTime, 0.0f, 0.0f);

            if (joystickValue.x > .5f) 
                robot.transform.Rotate(0f, speed * Time.fixedDeltaTime, 0.0f, Space.Self);
            
            if (joystickValue.x < -.5f)
                robot.transform.Rotate(0f, -speed * Time.fixedDeltaTime, 0.0f, Space.Self);

            robot.transform.Translate(0.0f, 0.0f, 0.0f);
        }
    }
}
