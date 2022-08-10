using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Menu : MonoBehaviour
{
    public GameObject m_locomotion;
    public GameObject m_homeButton;

    private Canvas _menu;
    
    void Start()
    {
        _menu = GetComponent<Canvas>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
