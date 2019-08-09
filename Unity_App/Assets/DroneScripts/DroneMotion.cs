using UnityEngine;
using System.Collections;

public class DroneMotion : MonoBehaviour {

   //--------------------------------------------------VARIABLE DECLARATIONS---------------------------------------------------------
    string messageToMC,droneh,dronef,droner,dronel;                                               // string variables
    string mcID = "HC-06";                                                                        // bluetooth module name
    Vector3 lookDir, moveDir;                                                                     // direction of looking within app and direction of motion within app
    float lookAngle, lookAngleZero;                                                               // look angle and reference which is set upon app start
    bool isGoing = false, started=false;                                                          // variable to descibe if we are travelling forward and variable to test start
    public float speed = 1.0f;                                                                    // speed that phone moves in VR, in the future postion needs to be updated by phone postions and not by this!
    float t,timebuffer;                                                                           // timer variables
    static float LOOKTHRESHOLD = 0.4f;                                                            // threshold which defines the angle at which you must look from the horizontal to move drone up and down
    double height;                                                                                // 

	//---------------------------------------------------INITIALIZATION---------------------------------------------------------------
	void Start () {
        t = Time.time;                                                                            // record start time
        timebuffer = Time.time;
        if (!BtConnector.isBluetoothEnabled())                                                    // if phone's bluetooth is disabled
            {BtConnector.askEnableBluetooth();}                                                   // prompt user to enable it
        BtConnector.moduleName(mcID);                                                             // tell bt connecter the name of the bt module we are going to connect to
        BtConnector.connect();                                                                    // connect phone to bt module
        droneh = "0"; dronef = "0"; droner = "0"; dronel = "0";                                   // set all controls initially to zero
        lookAngleZero = Cardboard.SDK.HeadPose.Orientation.eulerAngles.y;                         // set phone reference position
        height = 0;                                                                               // set initial height to zero
    }

    //---------------------------------------------------LOOP (is called once per frame)-----------------------------------------------
    void Update() {                                         // start check
        if (!started) {                                                                           // wait for start
            if (Cardboard.SDK.Triggered) {                                                        // check to see if user has tapped the screen, or pressed the vr button
                started = true;                                                                   // record that user has intialised the start
                droneh = "0";                                                                     // set initial height to 0   
            }                          
        }
        else {                                                                                    // once started
            lookDir = Cardboard.SDK.HeadPose.Orientation * Vector3.forward;                       // gets look direction vector
                                                               // forward
            if (Cardboard.SDK.Triggered && Time.time - t > 0.5) {                                 // wait for screen tap and 0.5s since last screen tap
                t = Time.time;                                                                    // record time of tap
                isGoing = !isGoing;                                                               // toggle between forward and hover
                if (isGoing) {                                                                    // if we want to go forward
                    dronef = "15"; }                                                              // set forward speed to 15
                else {                                                                            // if we want to hover
                    dronef = "0"; }                                                               // set forward speed to 0
            }
            if (isGoing) {                                                                        // if going forward
                moveDir = new Vector3(lookDir.x, 0.0f, lookDir.z);                                // puts movement only on the horizontal plane
                transform.position += moveDir * speed;                                            // move phone through VR environment by updating postion
            }
                                                                // height
            if (lookDir.y > LOOKTHRESHOLD && transform.position.y < 30) {                                                      // if phone looks up past threshold
                transform.position += Vector3.up * speed;                                         // move phone up in VR world by an amount depending on forward speed
                height += 30*speed;                                   // increment drone height  
                int heightInt = (int)height;
                droneh = heightInt.ToString();
            }                                                                                 
            else if (lookDir.y < -LOOKTHRESHOLD && transform.position.y >= 2) {                                                // if phone looks down past threshold
                transform.position -= Vector3.up * speed;                                         // move phone down in VR world by an amount depending on forward speed
                height -= 30*speed;                                   // decrement drone height
                int heightInt = (int)height;
                droneh = heightInt.ToString(); }                                                                             
                                                               // rotation
            lookAngle = Cardboard.SDK.HeadPose.Orientation.eulerAngles.y - lookAngleZero;         // retrieve phone looking angle
            lookAngle = Mathf.RoundToInt(lookAngle);                                              // round it to an integer
            if (lookAngle > 180) { lookAngle -= 360; }                                            // force angle in range [-180,180]
            else if (lookAngle < -180) { lookAngle += 360; }
            droner = lookAngle.ToString();                                                        // command drone rotation match phone rotation
                                                              // send commands
            if (Time.time - timebuffer > 0.1) {                                                   // every 0.1 seconds
                messageToMC = "," + droneh + "," + dronef + "," + droner + "," + dronel + ",";                // format instructions into form "height,forward,rotation,left"
       //         print(messageToMC);                                                               // print instuctions to unity console
                BtConnector.sendString(messageToMC);                                              // send instructions to drone
                timebuffer = Time.time;                                                           // record time of last message sent to drone
            }
        }
    }
}
