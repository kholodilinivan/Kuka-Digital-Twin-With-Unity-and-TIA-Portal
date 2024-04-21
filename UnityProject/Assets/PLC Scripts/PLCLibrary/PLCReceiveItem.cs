
using System;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.Events;


public class PLCReceiveItemGeneric<T> : PLCReceiveItem where T : struct, IEquatable<T>
{
    private T _value_t;
    public UnityEvent<T> OnGenericValueReceive;

    public override void ChangeState(object value)
    {
        base.ChangeState(value);
        if (value != null)
        {
            if(value is T)
            {
                var tempValue = (T)value;
                if (!EqualityComparer<T>.Default.Equals(tempValue, _value_t))
                {
                    _value_t = tempValue;
                    OnGenericValueReceive?.Invoke(tempValue);
                }
            }
            else
            {
                Debug.Log($"Выбран не правильный обработчик по типу, требуется: {value.GetType()}");
            }
        }
    }
}

public class PLCReceiveItem : MonoBehaviour
{
    public string data;
    public UnityEvent<object?> OnValueReceive;

    public virtual void ChangeState(object? value)
    {
        OnValueReceive?.Invoke(value);
    }
}
