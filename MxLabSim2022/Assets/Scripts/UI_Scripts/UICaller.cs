using UnityEngine.SceneManagement;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.UI;

public class UICaller : MonoBehaviour
{
    public InputActionAsset inputActions;
    public ActionBasedContinuousMoveProvider movement;
    
    Canvas UICanvas;
    InputAction menuButton;
    Scene currentScene;
    string sceneName;
    Button[] buttons;

    private void OnEnable()
    {
        UICanvas = GetComponent<Canvas>();
        buttons = GetComponentsInChildren<Button>();
        Scene currentScene = SceneManager.GetActiveScene();

        menuButton = inputActions.FindActionMap("XRI LeftHand UI").FindAction("Menu");
        menuButton.Enable();
        menuButton.performed += ToggleMenu;

        if (UICanvas.isActiveAndEnabled)
            UICanvas.enabled = !UICanvas.enabled;

        sceneName = currentScene.name;
        if (sceneName != "URScene")
        {
            for (int i = 0; i < buttons.Length; i++)
            {
                if (buttons[i].name == "AutoMove" || buttons[i].name == "Manual" || buttons[i].name == "Stop")
                    buttons[i].enabled = !buttons[i].enabled;
            }
        }
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
