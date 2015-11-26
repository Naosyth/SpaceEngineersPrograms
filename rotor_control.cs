// Block Names
string TextPanelName = "Screen One";
string RemoteControlName = "AH Remote";

Vector3D velocity = new Vector3D(10.0, 10.0, 10.0);

// Blocks
IMyTextPanel Screen;
IMyRemoteControl Remote;
List<IMyMotorStator> Rotors;

Dictionary<IMyMotorStator, Vector3D> oldPositions = new Dictionary<IMyMotorStator, Vector3D>();
Dictionary<IMyMotorStator, Vector3D> oldVelocities = new Dictionary<IMyMotorStator, Vector3D>();

void Main() {
  if (Remote == null)
    Initialize();

  AdjustRotors();
}

void Initialize() {
  if (!String.IsNullOrEmpty(TextPanelName))
    Screen = GridTerminalSystem.GetBlockWithName(TextPanelName) as IMyTextPanel;

  Remote = GridTerminalSystem.GetBlockWithName(RemoteControlName) as IMyRemoteControl;

  // Get all rotors on the ship
  var l = new List<IMyTerminalBlock>();
  GridTerminalSystem.GetBlocksOfType<IMyMotorStator>(l);
  Rotors = l.ConvertAll(x => (IMyMotorStator)x);
}

void AdjustRotors() {
  double dt = 1000 / 60;

  // Get ship's up vector
  Matrix orientation = Remote.WorldMatrix;
  Vector3 upVec = orientation.Up; upVec.Normalize();
  Vector3 rightVec = orientation.Right; rightVec.Normalize();
  Vector3 forwardVec = orientation.Forward; forwardVec.Normalize();
  Vector3 gravityVec = -Vector3.Normalize(Remote.GetNaturalGravity());

  // Iterate over each rotor
  for (int i = 0; i < Rotors.Count; i++) {
    // Get rotor's displacement derivative vector
    IMyMotorStator rotor = Rotors[i];
    Vector3D currentPos = rotor.GetPosition();
    Vector3D oldPos = new Vector3D(0, 0, 0);
    oldPositions.TryGetValue(rotor, out oldPos);
    Vector3D deltaPos = currentPos - oldPos;

    Vector3D currentVel = deltaPos / dt * 1000;
    Vector3D oldVel = new Vector3D(0, 0, 0);
    oldVelocities.TryGetValue(rotor, out oldVel);
    Vector3D deltaVel = currentVel - oldVel;

    Vector3D accVec = deltaVel * 1000 / dt;

    oldPositions[rotor] = currentPos;
    oldVelocities[rotor] = currentVel;

    // Get angle between ship's up vector and rotor's movement vector
    double speedForward = Vector3.Dot(accVec, forwardVec);
    double speedRight = Vector3.Dot(accVec, rightVec);
    double speedUp = Vector3.Dot(accVec, upVec);

    double pitch = Math.Atan2(speedForward, speedUp) * 180 / 3.14159; //Vector3D.Dot(accVec, forwardVec) / (accVec.Length() * forwardVec.Length()) * 90;
    double yaw = Math.Atan2(speedRight, speedUp) * 180 / 3.14159; //Vector3D.Dot(accVec, upVec) / (accVec.Length() * upVec.Length()) * 90;
    Vector3D rotationVec = new Vector3D(pitch, yaw, 0.0);

    Matrix localOrientation;
    rotor.Orientation.GetMatrix(out localOrientation);
    rotationVec = Vector3.Transform(rotationVec, localOrientation);

    // Get rotor's current angle
    int angle = GetRotorAngle(rotor);
    int desiredAngle = -(int)rotationVec.GetDim(1);

    if (accVec.Length() > 1) {
      float rotationVel = 30f * (angle < desiredAngle ? 1 : -1);
      rotationVel = (float)(Math.Atan(Math.Abs(angle-desiredAngle) / 2) / (Math.PI / 2) * rotationVel);

      if (Math.Abs(angle - desiredAngle) < 2)
        rotor.SetValueFloat("Velocity", 0f);
      else
        rotor.SetValueFloat("Velocity", rotationVel);
    } else {
      desiredAngle = angle;
      rotor.SetValueFloat("Velocity", 0f);
    }

    Screen.WritePublicText("Acc: " + accVec.Length() + 
      "\nVel: " + currentVel.Length() +
      "\nDesired Ang: " + desiredAngle +
      "\nCurrent Ang: " + angle + 
      "\nforward: " + speedForward +
      "\nright: " + speedRight + 
      "\nup: " + speedUp +
      "\npitch: " + pitch + 
      "\nyaw: " + yaw);

    // Scale thrust to thrusters on current rotor based on magnitude of displacement derivative vector
    // Get all thrusters attached to this rotor
    List<IMyThrust> Thrusters;
    var l = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyThrust>(l, x => x.CubeGrid != Me.CubeGrid);
    Thrusters = l.ConvertAll(x => (IMyThrust)x);

    for (int j = 0; j < Thrusters.Count; j++) {
      if (currentVel.Length() < 100 && Math.Abs(angle - desiredAngle) < 20)
        Thrusters[j].SetValueFloat("Override", (float)(Thrusters[j].GetMaximum<float>("Override") * (accVec.Length())));
      else
        Thrusters[j].SetValueFloat("Override", 0f);
    }

  }
}

int GetRotorAngle(IMyMotorStator rotor) {
  System.Text.RegularExpressions.Regex regex = new System.Text.RegularExpressions.Regex(@"-?[0-9]+");
  System.Text.RegularExpressions.Match match = regex.Match(rotor.DetailedInfo);
  if (match.Success)
    return Convert.ToInt32(match.Value);
  return 0;
}
