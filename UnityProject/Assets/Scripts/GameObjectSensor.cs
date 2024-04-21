using System;
using UnityEngine;
using UnityEngine.Events;

public class GameObjectSensor : MonoBehaviour
{
    public bool state = false;
    public float distance = 0.05f;
    public Vector3 direction;

    public UnityEvent<bool> OnSensorStateChange;

    public void Update()
    {
        Debug.DrawRay(transform.position, transform.TransformDirection(direction) * distance, Color.white);
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.TransformDirection(direction), out hit, distance))
        {
            Debug.DrawRay(transform.position, transform.TransformDirection(direction) * hit.distance, Color.yellow);
            if (hit.collider.tag == tag)
            {
                if(!state)
                {
                    state = true;
                    OnSensorStateChange?.Invoke(state);
                    Debug.Log("Did Hit");
                }
            }
        }
        else
        {
            Debug.DrawRay(transform.position, transform.TransformDirection(direction) * distance, Color.white);
            if(state)
            {
                state = false;
                OnSensorStateChange?.Invoke(state);
            }
        }
    }
}