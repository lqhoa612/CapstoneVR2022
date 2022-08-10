using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit;

public class WristUI : MonoBehaviour
{
    public InputActionAsset inputActions;
    public bool LocomotionSwitch;
    public ActionBasedContinuousMoveProvider _move;
    //public ActionBasedContinuousTurnProvider _turn;

    public GameObject defaultButton;

    private Canvas _UICanvas;
    private InputAction _menu;

    void Start()
    {
        _UICanvas = GetComponent<Canvas>();
        if (_UICanvas.isActiveAndEnabled)
            _UICanvas.enabled = !_UICanvas.enabled;
        _menu = inputActions.FindActionMap("XRI LeftHand UI").FindAction("Menu");
        _menu.Enable();
        _menu.performed += ToggleMenu;
    }

    void OnDestroy()
    {
        _menu.performed -= ToggleMenu;
    }

    public void ToggleMenu(InputAction.CallbackContext context)
    {
        _UICanvas.enabled = !_UICanvas.enabled;
        
        if (LocomotionSwitch == true)
        {
            _move.enabled = !_move.enabled;
            //_turn.enabled = !_turn.enabled;
        }

        EventSystem.current.SetSelectedGameObject(null);
        EventSystem.current.SetSelectedGameObject(defaultButton);
    }

}
