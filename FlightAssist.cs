// TODO
// * Enable/Disable Vector Control
// * Allow TransPose to have target vector or pitch/yaw/roll.
//   Once the target orientation is reached, clear target.
//     - Should it just be a target vector? Can I easily adapt AutoHover to set desiredVector instead of desired angles?  
// * Zero rotation values when transitioning gravity

// Configure which side of the ship has the main thrusters
// for flight in both gravity and space.
// Accepted values are 'bottom' and 'rear'
static string GravityMainThrust = "bottom";
static string SpaceMainThrust = "rear";

bool initialized = false;

FlightAssist fa;

const double HalfPi = Math.PI / 2;

void Main(string arguments) {
  if (!initialized) {
    fa = new FlightAssist(GridTerminalSystem, Me);

    initialized = true;
  }

  fa.Run(arguments);
}

class FlightAssist {
  List<Module> modules;

  public static IMyProgrammableBlock Me;
  public static IMyGridTerminalSystem GridTerminalSystem;

  public static TransPose transPose;
  public static AutoHover autoHover;
  public static VectorControl vectorControl;

  public FlightAssist(IMyGridTerminalSystem gts, IMyProgrammableBlock pb) {
    GridTerminalSystem = gts;
    Me = pb;

    modules = new List<Module>();
    modules.Add(transPose = new TransPose(this, "TransPose"));
    modules.Add(autoHover = new AutoHover(this, "AutoHover"));
    modules.Add(vectorControl = new VectorControl(this, "VectorControl"));
  }

  public void Run(string arguments) {
    string[] args = arguments.Split(' ');
    string moduleName = "";
    if (args.Length > 0)
      moduleName = args[0].ToLower();

    for (var i = 0; i < modules.Count; i++) {
      modules[i].Tick();
      if (modules[i].name.ToLower() == moduleName)
        modules[i].ProcessCommand(args);
    }
  }
}

abstract class Module {
  public string name;
  private FlightAssist FlightAssist;

  public Module(FlightAssist fa, string n) {
    FlightAssist = fa;
    name = n;
  }

  abstract public void Tick();
  abstract public void ProcessCommand(string[] args);
}

class TransPose : Module {
  private string gyroName = "FA Gyro";
  private string textPanelName = "Text panel 2"; // TODO: This does not belong in this module
  private string remoteControlName = "FA Remote";
  private int gyroCount = 1;

  private bool gyrosEnabled;

  public IMyRemoteControl remote;
  public IMyTextPanel screen;
  public IMyGyro gyro;
  private List<IMyGyro> gyros;

  private Vector3D oldPosition;
  public Vector3D position;
  public Vector3D deltaPosition;
  private Vector3 movementVector;

  public double speed;
  public double speedForward, speedRight, speedUp;

  private Matrix shipOrientation;
  private Matrix worldOrientation;
  private Vector3D forwardVec, rightVec, upVec;
  public double roll, pitch, yaw;

  public Vector3D gravity;
  public bool inGravity;
  public bool switchingGravity;

  public Vector3D targetOrientation;

  private double dt = 1000/60;

  public TransPose(FlightAssist fa, string n) : base(fa, n) {
    gyro = FlightAssist.GridTerminalSystem.GetBlockWithName(gyroName) as IMyGyro;

    var list = new List<IMyTerminalBlock>();
    FlightAssist.GridTerminalSystem.GetBlocksOfType<IMyGyro>(list, x => x.CubeGrid == FlightAssist.Me.CubeGrid && x != gyro);
    gyros = list.ConvertAll(x => (IMyGyro)x);
    gyros.Insert(0, gyro);
    gyros = gyros.GetRange(0, gyroCount);

    remote = FlightAssist.GridTerminalSystem.GetBlockWithName(remoteControlName) as IMyRemoteControl;

    if (!String.IsNullOrEmpty(textPanelName))
      screen = FlightAssist.GridTerminalSystem.GetBlockWithName(textPanelName) as IMyTextPanel;

    remote.Orientation.GetMatrix(out shipOrientation);
  }

  override public void Tick() {
    if (gyrosEnabled != gyro.GyroOverride)
      ToggleGyros(gyro.GyroOverride);

    CalcVelocity();
    CalcOrientation();
    CalcSpeedComponents();

    ApproachTargetOrientation();

    /*FlightAssist.transPose.screen.WritePublicText("Pitch: " + pitch +
                                                  "\nYaw: " + yaw +
                                                  "\nRoll: " + roll +
                                                  "\nSpeed Right: " + speedRight);*/
  }

  override public void ProcessCommand(string[] args) {}

  private void CalcVelocity() {
    position = remote.GetPosition();
    deltaPosition = position - oldPosition;
    oldPosition = position;
    speed = deltaPosition.Length() / dt * 1000;
    movementVector = Vector3D.Normalize(deltaPosition);
  }

  private void CalcOrientation() {
    gravity = -Vector3D.Normalize(remote.GetNaturalGravity());
    inGravity = !double.IsNaN(gravity.GetDim(0));
    switchingGravity = (inGravity != switchingGravity);

    worldOrientation = remote.WorldMatrix; // TODO: I should be able to transform this matrix, thus eliminating the need for SetOrientationVectors
    if (inGravity)
      SetOrientationVectors(GravityMainThrust);
    else
      SetOrientationVectors(SpaceMainThrust);

    if (inGravity) {
      pitch = Math.Asin(Vector3D.Dot(gravity, forwardVec) / (gravity.Length() * forwardVec.Length())) * (180/3.14159);
      roll = -Math.Asin(Vector3D.Dot(gravity, rightVec) / (gravity.Length() * rightVec.Length())) * (180/3.14159);
    } else {
      var t1 = Vector3D.Cross(movementVector, forwardVec);
      pitch = Math.Acos(Vector3D.Dot(movementVector, forwardVec) / (movementVector.Length() * forwardVec.Length())) * (180/3.14159);

      var t2 = Vector3D.Cross(movementVector, rightVec);
      yaw = Math.Acos(Vector3D.Dot(movementVector, rightVec) / (movementVector.Length() * rightVec.Length())) * (180/3.14159);
      if (speed == 0) {
        yaw = 0;
        pitch = 0;
      }
    }
  }

  private void CalcSpeedComponents() {
    if (inGravity) {
      speedForward = Vector3D.Dot(deltaPosition, Vector3D.Cross(gravity, rightVec)) / dt * 1000;
      speedRight = Vector3D.Dot(deltaPosition, -Vector3D.Cross(gravity, forwardVec)) / dt * 1000;
      speedUp = Vector3D.Dot(deltaPosition, gravity) / dt * 1000;
    } else {
      speedForward = Vector3D.Dot(deltaPosition, upVec) / dt * 1000;
      speedRight = Vector3D.Dot(deltaPosition, rightVec) / dt * 1000;
      speedUp = Vector3D.Dot(deltaPosition, forwardVec) / dt * 1000;
    }
  }

  private float ClampMinRpm(float val) {
    var minRpm = 0.015f;
    if (val > -minRpm && val < minRpm)
      return (float)(Math.Sign(val) * minRpm);
    else
      return val;
  }

  private double ShortestDistToAngle(double current, double target) {
    return Math.Atan2(Math.Sin(target*(Math.PI/180) - current*(Math.PI/180)), Math.Cos(target*(Math.PI/180) - current*(Math.PI/180))) * (180/3.14159);
  }

  private void SetOrientationVectors(string direction) {
    if (direction == "bottom") {
      forwardVec = worldOrientation.Forward;
      rightVec = worldOrientation.Right;
      upVec = worldOrientation.Up;
    } else if (direction == "rear") {
      forwardVec = worldOrientation.Up;
      rightVec = worldOrientation.Right;
      upVec = worldOrientation.Forward;
    }
  }

  public void ToggleGyros(bool state) {
    gyrosEnabled = state;
    for (int i = 0; i < gyros.Count; i++)
      gyros[i].SetValueBool("Override", gyrosEnabled);
  }

  private void ApproachTargetOrientation() {
    // Transform rotation to match the remote control block's orientation rather than the "build" orientation
    Vector3D rotationVector = Vector3.Transform(targetOrientation, shipOrientation);

    float pitchRate = ClampMinRpm((float)(FlightAssist.transPose.gyro.GetMaximum<float>("Pitch") * (ShortestDistToAngle(pitch, targetOrientation.GetDim(0)) / 360)));
    float yawRate = ClampMinRpm((float)(FlightAssist.transPose.gyro.GetMaximum<float>("Yaw") * (ShortestDistToAngle(yaw, targetOrientation.GetDim(1)) / 360)));
    FlightAssist.transPose.screen.WritePublicText("Pitch: " + pitch +
                                                  "\nTarget: " + targetOrientation.GetDim(0) +
                                                  "\nDist: " + ShortestDistToAngle(pitch, targetOrientation.GetDim(0)) +
                                                  "\nRate: " + pitchRate +
                                                  "\n\nYaw: " + yaw +
                                                  "\nTarget: " + targetOrientation.GetDim(1) +
                                                  "\nDist: " + ShortestDistToAngle(yaw, targetOrientation.GetDim(1)) +
                                                  "\nRate: " + yawRate);
    float rollRate = ClampMinRpm((float)(FlightAssist.transPose.gyro.GetMaximum<float>("Roll") * (ShortestDistToAngle(roll, targetOrientation.GetDim(2)) / 360)));

    for (int i = 0; i < gyros.Count; i++) {
      var g = gyros[i];

      // Adjust rotation for the gyro's local orientation
      Matrix localOrientation;
      g.Orientation.GetMatrix(out localOrientation);
      var localRot = Vector3.Transform(rotationVector, MatrixD.Transpose(localOrientation));

      g.SetValueFloat("Pitch", pitchRate);
      g.SetValueFloat("Yaw", yawRate);
      g.SetValueFloat("Roll", 0);
    }
  }
}

class VectorControl : Module {
  private float pitchRate, yawRate;

  public VectorControl(FlightAssist fa, string n) : base(fa, n) {}

  override public void Tick() {
    ExecuteManeuver();
  }

  override public void ProcessCommand(string[] args) {}

  private void ExecuteManeuver() {
    /*if (FlightAssist.transPose.speedForward > 0) {
      pitchRate = FlightAssist.transPose.gyro.GetMaximum<float>("Pitch");
      yawRate = FlightAssist.transPose.gyro.GetMaximum<float>("Yaw");
    } else {
      pitchRate = (float)((FlightAssist.transPose.gyro.GetMaximum<float>("Pitch") * (FlightAssist.transPose.pitch) / 90));
      yawRate = (float)((FlightAssist.transPose.gyro.GetMaximum<float>("Yaw") * (-FlightAssist.transPose.yaw) / 90));
    }

    if (FlightAssist.transPose.speed < 0.5) {
      yawRate = 0;
      pitchRate = 0;
    }*/

    FlightAssist.transPose.targetOrientation = new Vector3D(0, 0, 0);
  }
}


class AutoHover : Module {
  private string mode;

  private double desiredPitch, desiredRoll;
  private float pitchRate, rollRate;
  private float setSpeed;

  private double maxPitch = 67.5;
  private double maxRoll = 67.5;
  private int gyroResponsiveness = 16; // Larger = more gradual angle drop

  public AutoHover(FlightAssist fa, string n) : base(fa, n) {
    mode = "Hover";
  }

  override public void Tick() {
    if (!FlightAssist.transPose.inGravity)
      return;

    //ExecuteManeuver();
  }

  override public void ProcessCommand(string[] args) {
    if (args == null)
      return;

    mode = args[1];
  }

  private void ExecuteManeuver() {
    switch (mode.ToLower()) {
      case "glide":
        desiredPitch = 0;
        desiredRoll = Math.Atan(FlightAssist.transPose.speedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      case "freeglide":
        desiredPitch = 0;
        desiredRoll = 0;
        break;

      case "pitch":
        desiredPitch = Math.Atan(FlightAssist.transPose.speedForward / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = FlightAssist.transPose.roll;
        break;

      case "roll":
        desiredPitch = FlightAssist.transPose.pitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.speedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      case "cruise":
        desiredPitch = Math.Atan((FlightAssist.transPose.speedForward - setSpeed) / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.speedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      default: // Stationary Hover
        desiredPitch = Math.Atan(FlightAssist.transPose.speedForward / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.speedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;
    }

    // Scale gyro rate based on difference bewteen the current and desired angle
    pitchRate = (float)((FlightAssist.transPose.gyro.GetMaximum<float>("Pitch") * (desiredPitch - FlightAssist.transPose.pitch) / 90));
    rollRate = (float)((FlightAssist.transPose.gyro.GetMaximum<float>("Roll") * (-desiredRoll - FlightAssist.transPose.roll) / 90));

    //FlightAssist.transPose.SetGyros(new Vector3(pitchRate, 0, rollRate));
  }
}
