using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class URMovement : MonoBehaviour
{
    InputActionAsset playerControl;
    InputAction movement;

    GameObject robot;
    //[SerializeField] float speed = 10.0f;

    // Start is called before the first frame update
    void Start()
    {
        var gameplayActionMap = playerControl.FindActionMap("LeftHand");
        movement = gameplayActionMap.FindAction("PrimaryButton");
        //movement.performed += RotationDirection();
        robot = GetComponent<GameObject>();
    }

    // Update is called once per frame
    void Update()
    {
    }

    void OnMovementChange(InputAction.CallbackContext context)
    {
        Vector2 direction = context.ReadValue<Vector2>();
    }
}
