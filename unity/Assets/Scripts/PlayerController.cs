using System.Collections;
using System.Collections.Generic;
using UnityEngine;

 public enum State
 {
    walking, running
 }

public class PlayerController : MonoBehaviour
{
    [Range(0,60)]
    public int startDelay;
    public bool crouched = false;

    public bool loop = false;
    public State actionType ;

    private Animator anim;
    private int _state = -1;
    private int _prev_state = -2;

    private bool _turning = false;
    private float _distance_forward = 0;
    private Vector3 _position;

    private double _start_time;
    

    void Start()
    {
        // Get the animator from this frame
        anim = GetComponent<Animator>();
        // Get the start time
        _start_time = Time.time;
    }

    // Update is called once per frame
    void Update()
    {
        AnimatorStateInfo stateInfo = anim.GetCurrentAnimatorStateInfo(0);
        AnimatorTransitionInfo transInfo = anim.GetAnimatorTransitionInfo(0);

        // Wait to start
        if (Time.time - _start_time > startDelay)
        {
            if (actionType == State.walking)
            {
                update_walk(stateInfo, transInfo);
            }

            if (actionType == State.running)
            {
                update_run(stateInfo, transInfo);
            }

            // Check if we had any turning animation started
            if (_turning)
            {
                checkTurns(transInfo);
            }

            // Check if we have reached our destination forward
            if (_distance_forward > 0)
            {
                checkForward(stateInfo);
            }
        }
    }

    // Used for walking
    void update_walk(AnimatorStateInfo stateInfo, AnimatorTransitionInfo transInfo)
    {
        // If we have a new state
        if (_state != _prev_state)
        {
            // Remember that we have dealt with the new state
            _prev_state = _state;
            // Process the new state
            switch(_state)
            {
                case -1:
                    anim.SetBool("crouch", crouched);
                    _state = 0;
                    break;
                case 0:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 3;
                    _position = transform.position;
                    break;
                case 1:
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_right", true);
                    _turning = true;
                    break;
                case 2:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 6;
                    _position = transform.position;
                    break;
                case 3:
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_right", true);
                    _turning = true;
                    break;
                case 4:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 16;
                    _position = transform.position;
                    break;
                case 5:
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_right", true);
                    _turning = true;
                    break;
                case 6:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 12;
                    _position = transform.position;
                    break;
                case 7:
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_right", true);
                    _turning = true;
                    break;
                case 8:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 13;
                    _position = transform.position;
                    break;
                case 9:
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_right", true);
                    _turning = true;
                    break;
                case 10:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 6;
                    _position = transform.position;
                    break;
                case 11:
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_left", true);
                    _turning = true;
                    break;
                case 12:
                    if (loop)
                    {
                        _state = -1;
                    }
                    break;
            }
        }
    }

    // Used for running
    void update_run(AnimatorStateInfo stateInfo, AnimatorTransitionInfo transInfo)
    {
        // If we have a new state
        if (_state != _prev_state)
        {
            // Remember that we have dealt with the new state
            _prev_state = _state;
            // Process the new state
            switch(_state)
            {
                case -1:
                    _state = 0;
                    anim.SetBool("crouch", crouched);
                    break;
                case 0:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 3;
                    _position = transform.position;
                    break;
                case 1:
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_right", true);
                    _turning = true;
                    break;
                case 2:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 6;
                    _position = transform.position;
                    break;
                case 3:
                    Debug.Log("First turn left");
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_left", true);
                    _turning = true;
                    break;
                case 4:
                    if(stateInfo.IsName("Stand") || stateInfo.IsName("Crouch"))
                    {
                        _state = _state + 1;
                    }    
                    else
                    {
                        _prev_state = _state - 1;
                    }
                    break;
                case 5:
                    Debug.Log("Second turn left");
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_left", true);
                    _turning = true;
                    break;
                case 6:
                    Debug.Log("Running");
                    anim.SetFloat("forward", 0.65f);
                    _distance_forward = 6;
                    _position = transform.position;
                    break;
                case 7:
                    anim.SetFloat("forward", 0.0f);
                    anim.SetBool("turn_left", true);
                    _turning = true;
                    break;
                case 8:
                    anim.SetFloat("forward", 0.4f);
                    _distance_forward = 3;
                    _position = transform.position;
                    break;
                case 9:
                    if (loop)
                    {
                        _state = -1;
                    }
                    break;
            }
        }        
    }

    void checkTurns(AnimatorTransitionInfo transInfo)
    {
        bool left_to_stand   = (transInfo.nameHash == Animator.StringToHash("TurnLeft -> Stand"));
        bool left_to_crouch  = (transInfo.nameHash == Animator.StringToHash("CrouchTurnLeft -> Crouch"));
        bool right_to_stand  = (transInfo.nameHash == Animator.StringToHash("TurnRight -> Stand"));
        bool right_to_crouch = (transInfo.nameHash == Animator.StringToHash("CrouchTurnRight -> Crouch"));

        // Check if we have finished the rotation
        if ((left_to_stand) || (left_to_crouch))
        {
            anim.SetBool("turn_left", false);
            if (_turning == true)
            {
                _turning = false;
                _state = _state + 1;
            }
        }
        if ((right_to_stand) || (right_to_crouch))
        {
            anim.SetBool("turn_right", false);
            if (_turning == true)
            {
                _turning = false;
                _state = _state + 1;
                fixRotation();
            }
        }
    }

    void checkForward(AnimatorStateInfo stateInfo)
    {
        float stopDelay = 0.0f;
        if (stateInfo.IsName("Walk"))
        {
            stopDelay = 0.65f;
        }
        if (stateInfo.IsName("Jog"))
        {
            stopDelay = 1f;
        }
        
        float dist = Vector3.Distance(_position, transform.position);

        if (dist > (_distance_forward - stopDelay))
        {
            _distance_forward = 0;
            _state = _state + 1;
        }
    }

    void fixRotation()
    {
        float x = transform.rotation.eulerAngles.x ;
        float y = transform.rotation.eulerAngles.y ;
        float z = transform.rotation.eulerAngles.z ;
        Quaternion target = Quaternion.Euler(Mathf.Round(x), Mathf.Round(y), Mathf.Round(z)) ;
        transform.rotation = target ;
    }
}
