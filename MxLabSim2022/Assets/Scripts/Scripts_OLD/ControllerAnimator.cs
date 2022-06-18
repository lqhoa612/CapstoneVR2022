using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;

[RequireComponent(typeof(Animator))]
public class ControllerAnimator : MonoBehaviour
{
    Animator _animator;
    private readonly XRNode _controller;
    private readonly List<InputDevice> _deviceList = new List<InputDevice>();
    private InputDevice _device;

    void GetDevice()
    {
        InputDevices.GetDevicesAtXRNode(_controller, _deviceList);
        _device = _deviceList.FirstOrDefault();
    }

    // Start is called before the first frame update
    void Start()
    {
        _animator = GetComponent<Animator>();
        if (!_device.isValid)
            GetDevice();
    }

    // Update is called once per frame
    void Update()
    {
        if (!_device.isValid)
            GetDevice();

        AnimateController();
    }

    private void AnimateController()
    {
        if (_animator != null)
        {
            InputFeatureUsage<bool> primaryUsage = CommonUsages.primaryButton;
            _device.TryGetFeatureValue(primaryUsage, out bool primaryValue);
            _animator.SetBool("Button 1", primaryValue);

            InputFeatureUsage<bool> secondaryUsage = CommonUsages.secondaryButton;
            _device.TryGetFeatureValue(secondaryUsage, out bool secondaryValue);
            _animator.SetBool("Button 2", secondaryValue);

            InputFeatureUsage<bool> menuUsage = CommonUsages.menuButton;
            _device.TryGetFeatureValue(menuUsage, out bool menuValue);
            _animator.SetBool("Button 3", menuValue);

            InputFeatureUsage<Vector2> joyUsage = CommonUsages.primary2DAxis;
            _device.TryGetFeatureValue(joyUsage, out Vector2 joyValue);
            _animator.SetFloat("Joy X", joyValue.x);
            _animator.SetFloat("Joy Y", joyValue.y);

            InputFeatureUsage<float> triggerUsage = CommonUsages.trigger;
            _device.TryGetFeatureValue(triggerUsage, out float triggerValue);
            _animator.SetFloat("Trigger", triggerValue);

            InputFeatureUsage<float> gripUsage = CommonUsages.grip;
            _device.TryGetFeatureValue(gripUsage, out float gripValue);
            _animator.SetFloat("Grip", gripValue);
        }
    }
}
