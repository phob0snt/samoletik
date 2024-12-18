using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(PlaneController))]
public class PlayerController : MonoBehaviour, PlayerInputAsset.IPlayerInputActions
{
    private PlaneController _planeController => GetComponent<PlaneController>();

    private PlayerInputAsset _playerInput;
    private Vector3 _controlInput;

    void Awake()
    {
        _playerInput = new PlayerInputAsset();

        _playerInput.PlayerInput.SetCallbacks(this);
    }

    private void Update()
    {
        if (_planeController == null)
        {
            return;
        }

        _planeController.SetControlInput(_controlInput);
    }

    void OnEnable()
    {
        _playerInput.PlayerInput.Enable();
    }

    void OnDisable()
    {
        _playerInput.PlayerInput.Disable();
    }

    public void OnThrottle(InputAction.CallbackContext context)
    {
        if (_planeController == null)
        {
            return;
        }

        _planeController.SetThrottleInput(context.ReadValue<float>());
    }

    public void OnFlaps(InputAction.CallbackContext context)
    {
        if (_planeController == null)
        {
            return;
        }

        if (context.phase == InputActionPhase.Performed)
        {
            _planeController.ToggleFlaps();
        }
    }

    public void OnRollPitch(InputAction.CallbackContext context)
    {
        if (_planeController == null)
        {
            return;
        }

        var input = context.ReadValue<Vector2>();

        _controlInput = new Vector3(input.y, _controlInput.y, -input.x);
    }

    public void OnYaw(InputAction.CallbackContext context)
    {
        if (_planeController == null)
        {
            return;
        }

        var input = context.ReadValue<float>();

        _controlInput = new Vector3(_controlInput.x, input, _controlInput.z);
    }
}