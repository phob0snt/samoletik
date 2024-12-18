//------------------------------------------------------------------------------
// <auto-generated>
//     This code was auto-generated by com.unity.inputsystem:InputActionCodeGenerator
//     version 1.11.2
//     from Assets/PlayerInputAsset.inputactions
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Utilities;

public partial class @PlayerInputAsset: IInputActionCollection2, IDisposable
{
    public InputActionAsset asset { get; }
    public @PlayerInputAsset()
    {
        asset = InputActionAsset.FromJson(@"{
    ""name"": ""PlayerInputAsset"",
    ""maps"": [
        {
            ""name"": ""PlayerInput"",
            ""id"": ""55115dd5-dd46-4584-ab05-6baf33b7216a"",
            ""actions"": [
                {
                    ""name"": ""Throttle"",
                    ""type"": ""Value"",
                    ""id"": ""cfc61433-117a-4d7a-a93b-347f2fe5d992"",
                    ""expectedControlType"": """",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": true
                },
                {
                    ""name"": ""Flaps"",
                    ""type"": ""Button"",
                    ""id"": ""5351c36f-5e85-4bd4-99fe-d3c88229916a"",
                    ""expectedControlType"": ""Button"",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": false
                },
                {
                    ""name"": ""RollPitch"",
                    ""type"": ""Value"",
                    ""id"": ""8d6575e4-9b7b-4fc3-bf22-c0ab8ae3b1c1"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": true
                },
                {
                    ""name"": ""Yaw"",
                    ""type"": ""Value"",
                    ""id"": ""1c2cbb93-e1b2-4eb2-89b4-e9119eb5b88a"",
                    ""expectedControlType"": """",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": true
                }
            ],
            ""bindings"": [
                {
                    ""name"": ""1D Axis"",
                    ""id"": ""68c4af96-915c-4bdd-a669-abe1b3671127"",
                    ""path"": ""1DAxis"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Throttle"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""positive"",
                    ""id"": ""1562d343-a9f8-4ff9-ad5e-318284a0f59d"",
                    ""path"": ""<Keyboard>/shift"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Throttle"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""negative"",
                    ""id"": ""c406f3ce-6276-4709-ba9a-13e1be7c91bc"",
                    ""path"": ""<Keyboard>/ctrl"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Throttle"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": """",
                    ""id"": ""9434b7f0-5632-4b8a-8271-1126dac25d43"",
                    ""path"": ""<Keyboard>/f"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Flaps"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""2D Vector"",
                    ""id"": ""5f2064d3-563d-49a9-9384-1ce7a87d77cd"",
                    ""path"": ""2DVector"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""RollPitch"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""up"",
                    ""id"": ""a8d13a0e-1fe1-483e-978d-e95ac69b884b"",
                    ""path"": ""<Keyboard>/w"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""RollPitch"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""down"",
                    ""id"": ""6c600736-577f-47dc-a45f-f719bfe3cdef"",
                    ""path"": ""<Keyboard>/s"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""RollPitch"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""left"",
                    ""id"": ""09ab67b4-38b6-4b2d-9358-e8918ac3659b"",
                    ""path"": ""<Keyboard>/a"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""RollPitch"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""right"",
                    ""id"": ""bfc39433-36df-49b9-b2bf-4783a9d799ef"",
                    ""path"": ""<Keyboard>/d"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""RollPitch"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""1D Axis"",
                    ""id"": ""84170057-cec8-440c-ad39-150eecb06e61"",
                    ""path"": ""1DAxis"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Yaw"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""positive"",
                    ""id"": ""f7c0d7ce-128b-4351-a40b-96a40590087a"",
                    ""path"": ""<Keyboard>/e"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Yaw"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""negative"",
                    ""id"": ""70922c03-d415-4263-aeb0-199d80840c88"",
                    ""path"": ""<Keyboard>/q"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Yaw"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                }
            ]
        }
    ],
    ""controlSchemes"": []
}");
        // PlayerInput
        m_PlayerInput = asset.FindActionMap("PlayerInput", throwIfNotFound: true);
        m_PlayerInput_Throttle = m_PlayerInput.FindAction("Throttle", throwIfNotFound: true);
        m_PlayerInput_Flaps = m_PlayerInput.FindAction("Flaps", throwIfNotFound: true);
        m_PlayerInput_RollPitch = m_PlayerInput.FindAction("RollPitch", throwIfNotFound: true);
        m_PlayerInput_Yaw = m_PlayerInput.FindAction("Yaw", throwIfNotFound: true);
    }

    ~@PlayerInputAsset()
    {
        UnityEngine.Debug.Assert(!m_PlayerInput.enabled, "This will cause a leak and performance issues, PlayerInputAsset.PlayerInput.Disable() has not been called.");
    }

    public void Dispose()
    {
        UnityEngine.Object.Destroy(asset);
    }

    public InputBinding? bindingMask
    {
        get => asset.bindingMask;
        set => asset.bindingMask = value;
    }

    public ReadOnlyArray<InputDevice>? devices
    {
        get => asset.devices;
        set => asset.devices = value;
    }

    public ReadOnlyArray<InputControlScheme> controlSchemes => asset.controlSchemes;

    public bool Contains(InputAction action)
    {
        return asset.Contains(action);
    }

    public IEnumerator<InputAction> GetEnumerator()
    {
        return asset.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }

    public void Enable()
    {
        asset.Enable();
    }

    public void Disable()
    {
        asset.Disable();
    }

    public IEnumerable<InputBinding> bindings => asset.bindings;

    public InputAction FindAction(string actionNameOrId, bool throwIfNotFound = false)
    {
        return asset.FindAction(actionNameOrId, throwIfNotFound);
    }

    public int FindBinding(InputBinding bindingMask, out InputAction action)
    {
        return asset.FindBinding(bindingMask, out action);
    }

    // PlayerInput
    private readonly InputActionMap m_PlayerInput;
    private List<IPlayerInputActions> m_PlayerInputActionsCallbackInterfaces = new List<IPlayerInputActions>();
    private readonly InputAction m_PlayerInput_Throttle;
    private readonly InputAction m_PlayerInput_Flaps;
    private readonly InputAction m_PlayerInput_RollPitch;
    private readonly InputAction m_PlayerInput_Yaw;
    public struct PlayerInputActions
    {
        private @PlayerInputAsset m_Wrapper;
        public PlayerInputActions(@PlayerInputAsset wrapper) { m_Wrapper = wrapper; }
        public InputAction @Throttle => m_Wrapper.m_PlayerInput_Throttle;
        public InputAction @Flaps => m_Wrapper.m_PlayerInput_Flaps;
        public InputAction @RollPitch => m_Wrapper.m_PlayerInput_RollPitch;
        public InputAction @Yaw => m_Wrapper.m_PlayerInput_Yaw;
        public InputActionMap Get() { return m_Wrapper.m_PlayerInput; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled => Get().enabled;
        public static implicit operator InputActionMap(PlayerInputActions set) { return set.Get(); }
        public void AddCallbacks(IPlayerInputActions instance)
        {
            if (instance == null || m_Wrapper.m_PlayerInputActionsCallbackInterfaces.Contains(instance)) return;
            m_Wrapper.m_PlayerInputActionsCallbackInterfaces.Add(instance);
            @Throttle.started += instance.OnThrottle;
            @Throttle.performed += instance.OnThrottle;
            @Throttle.canceled += instance.OnThrottle;
            @Flaps.started += instance.OnFlaps;
            @Flaps.performed += instance.OnFlaps;
            @Flaps.canceled += instance.OnFlaps;
            @RollPitch.started += instance.OnRollPitch;
            @RollPitch.performed += instance.OnRollPitch;
            @RollPitch.canceled += instance.OnRollPitch;
            @Yaw.started += instance.OnYaw;
            @Yaw.performed += instance.OnYaw;
            @Yaw.canceled += instance.OnYaw;
        }

        private void UnregisterCallbacks(IPlayerInputActions instance)
        {
            @Throttle.started -= instance.OnThrottle;
            @Throttle.performed -= instance.OnThrottle;
            @Throttle.canceled -= instance.OnThrottle;
            @Flaps.started -= instance.OnFlaps;
            @Flaps.performed -= instance.OnFlaps;
            @Flaps.canceled -= instance.OnFlaps;
            @RollPitch.started -= instance.OnRollPitch;
            @RollPitch.performed -= instance.OnRollPitch;
            @RollPitch.canceled -= instance.OnRollPitch;
            @Yaw.started -= instance.OnYaw;
            @Yaw.performed -= instance.OnYaw;
            @Yaw.canceled -= instance.OnYaw;
        }

        public void RemoveCallbacks(IPlayerInputActions instance)
        {
            if (m_Wrapper.m_PlayerInputActionsCallbackInterfaces.Remove(instance))
                UnregisterCallbacks(instance);
        }

        public void SetCallbacks(IPlayerInputActions instance)
        {
            foreach (var item in m_Wrapper.m_PlayerInputActionsCallbackInterfaces)
                UnregisterCallbacks(item);
            m_Wrapper.m_PlayerInputActionsCallbackInterfaces.Clear();
            AddCallbacks(instance);
        }
    }
    public PlayerInputActions @PlayerInput => new PlayerInputActions(this);
    public interface IPlayerInputActions
    {
        void OnThrottle(InputAction.CallbackContext context);
        void OnFlaps(InputAction.CallbackContext context);
        void OnRollPitch(InputAction.CallbackContext context);
        void OnYaw(InputAction.CallbackContext context);
    }
}
