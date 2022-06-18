using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.XR;

public class SceneSelector : MonoBehaviour
{
    public GameObject _sceneList;
    public GameObject _defaultButton;
    private Canvas _UICanvas;

    private readonly XRNode sceneControlNode = XRNode.LeftHand;
    private readonly List<InputDevice> deviceList = new List<InputDevice>();
    private InputDevice device;

    private void Start()
    {
        _UICanvas = GetComponent<Canvas>();
        if (_UICanvas.isActiveAndEnabled == true)
            _UICanvas.enabled = !_UICanvas.enabled;
    }

    void GetDevice()
    {
        InputDevices.GetDevicesAtXRNode(sceneControlNode, deviceList);
        device = deviceList.FirstOrDefault();
    }

    private void OnEnable()
    {
        if (!device.isValid)
            GetDevice();
    }

    // Update is called once per frame
    void Update()
    {
        if (!device.isValid)
            GetDevice();

        InputFeatureUsage<bool> menuUsage = CommonUsages.menuButton;
        device.TryGetFeatureValue(menuUsage, out bool menuValue);
        if (menuValue == true && _UICanvas.isActiveAndEnabled == false)
        {
            _UICanvas.enabled = _UICanvas.enabled;
            //SceneSelectionUI();
        }

        if (menuValue == true && _UICanvas.isActiveAndEnabled == true)
        {
            _UICanvas.enabled = !_UICanvas.enabled;
        }



    }

    public void SceneSelectionUI()
    {
        if (!_sceneList.activeInHierarchy)
        {
            _sceneList.SetActive(true);
            Time.timeScale = 0f;
            //clear selected button and set new selected button
            EventSystem.current.SetSelectedGameObject(null);
            EventSystem.current.SetSelectedGameObject(_defaultButton);
        }
        else
        {
            _sceneList.SetActive(false);
            Time.timeScale = 1f;
        }
    }
}
