using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    [HideInInspector]
    public Animator anim;
    // public GameObject personObject;

    private int _state = -1;
    private bool _turning = false;
    private double start_time;

    void Start()
    {
        // Get the animator from this frame
        anim = GetComponent<Animator>();
        // Get the start time
        start_time = Time.time;
    }

    // Update is called once per frame
    void Update()
    {
        AnimatorStateInfo stateInfo = anim.GetCurrentAnimatorStateInfo(0);
        AnimatorTransitionInfo transInfo = anim.GetAnimatorTransitionInfo(0);

        // Wait 15 seconds before starting
        if ((_state == -1) && (Time.time - start_time > 15))
        {
            _state = 0;
        }

        if (_state == 0)
        {
            anim.SetFloat("forward", 0.4f);
        }
        
        if ((_state == 0) && (transform.position.z > 8))
        {
            anim.SetFloat("forward", 0.0f);
            anim.SetBool("turn_right", true);
            _state = _state + 1;
            _turning = true;
        }

        if (_state == 2)
        {
            anim.SetFloat("forward", 0.65f);
        }

        if ((_state == 2) && (transform.position.x > 6))
        {
            anim.SetFloat("forward", 0.0f);
            if (stateInfo.IsName("Stand"))
            {
                anim.SetBool("turn_left", true);
                _state = _state + 1;
                _turning = true;
            }
        }

        if (_state == 4)
        {
            if (stateInfo.IsName("Stand"))
            {
                anim.SetBool("turn_left", true);
                _state = _state + 1;
                _turning = true;
            }
        }

        if (_state == 6)
        {
            anim.SetFloat("forward", 0.65f);
        }

        if ((_state == 6) && (transform.position.x < 0))
        {
            anim.SetFloat("forward", 0.35f);
            _state = _state + 1;
        }

        if ((_state == 7) && (transform.position.x < -2))
        {
            anim.SetFloat("forward", 0.0f);
            anim.SetBool("crouch", true);
        }

        // Check if we have finished the rotation
        
        if(transInfo.nameHash == Animator.StringToHash("Turn Left -> Stand"))
        {
            anim.SetBool("turn_left", false);
            if (_turning == true)
            {
                _turning = false;
                _state = _state + 1;
            }
        }
        if(transInfo.nameHash == Animator.StringToHash("Turn Right -> Stand"))
        {
            anim.SetBool("turn_right", false);
            if (_turning == true)
            {
                _turning = false;
                _state = _state + 1;
            }
        }
    }
}
