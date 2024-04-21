
using System;

using UnityEngine;

public class PLCSendItem : MonoBehaviour
{
    public string data;
    public event Action<object?, string> OnValueChange;

    public void ChangeState(object? value)
    {
        OnValueChange?.Invoke(value, data);
    }
}
