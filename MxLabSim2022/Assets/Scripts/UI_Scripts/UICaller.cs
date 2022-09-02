using UnityEngine.SceneManagement;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit;

public class UICaller : MonoBehaviour
{
    public InputActionAsset inputActions;
    public ActionBasedContinuousMoveProvider movement;
    
    Canvas UICanvas;
    InputAction menuButton;
    Scene currentScene;
    string sceneName;

    private void OnEnable()
    {
        UICanvas = GetComponent<Canvas>();
        if (UICanvas.isActiveAndEnabled)
            UICanvas.enabled = !UICanvas.enabled;
        menuButton = inputActions.FindActionMap("XRI LeftHand UI").FindAction("Menu");
        menuButton.Enable();
        menuButton.performed += ToggleMenu;
        currentScene = SceneManager.GetActiveScene();
        sceneName = currentScene.name;
    }

    private void OnDestroy()
    {
        menuButton.performed -= ToggleMenu;
    }

    void ToggleMenu(InputAction.CallbackContext context)
    {
        UICanvas.enabled = !UICanvas.enabled;
        if (movement.enabled == true && sceneName == "MainScene")
            movement.enabled = !movement.enabled;
    }
}
