using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;

public class UGVControllerInput : MonoBehaviour
{
    public GameObject robot;
    [SerializeField] private float speed = 10f;

    private XRNode rightNode = XRNode.RightHand;
    private readonly List<InputDevice> devices = new List<InputDevice>();
    private InputDevice device;
    // Start is called before the first frame update
    void GetDevice()
    {
        InputDevices.GetDevicesAtXRNode(rightNode, devices);
        device = devices.FirstOrDefault();
    }

    void OnEnable()
    {
        if (!device.isValid)
            GetDevice();
    }

    // Update is called once per frame
    private void FixedUpdate()
    {
        if (!device.isValid)
            GetDevice();

        InputFeatureUsage<Vector2> joystickUsage = CommonUsages.primary2DAxis;

        if (device.TryGetFeatureValue(joystickUsage, out Vector2 joystickValue))
        {
            if (joystickValue.y > .5f) 
                robot.transform.Translate(-speed/50 * Time.fixedDeltaTime, 0f, 0f);
                
            if (joystickValue.y < -.5f)
                robot.transform.Translate(speed/50 * Time.fixedDeltaTime, 0f, 0f);

            if (joystickValue.x > .5f) 
                robot.transform.Rotate(0f, speed * Time.fixedDeltaTime, 0f, Space.Self);
            
            if (joystickValue.x < -.5f)
                robot.transform.Rotate(0f, -speed * Time.fixedDeltaTime, 0f, Space.Self);

            robot.transform.Translate(0f, 0f, 0f);
        }
    }
}
