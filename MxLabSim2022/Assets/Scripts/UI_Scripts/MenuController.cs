using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MenuController : MonoBehaviour
{
    public WristUI m_wristUI;
    public XRControllerCapture m_XRCapture;

    private void OnEnable()
    {
        if (m_wristUI.isActiveAndEnabled)
            m_wristUI.enabled = false;
    }

    private void Update()
    {
        if (m_XRCapture.XisPressed == true)
        {
            m_wristUI.enabled = true;
        }
    }
}
