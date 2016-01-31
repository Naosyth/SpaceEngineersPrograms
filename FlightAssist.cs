// TODO
// * Enable/Disable Vector Control
// * Allow TransPose to have target vector or pitch/yaw/roll.
//   Once the target orientation is reached, clear target.
//     - Should it just be a target vector? Can I easily adapt AutoHover to set desiredVector instead of desired angles?  
// * Zero rotation values when transitioning gravity
// * Try axis angle -> euler, otherwise orient via vectors and fudge the gyro values.

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
  public static HoverAssist hoverAssist;
  public static VectorAssist vectorAssist;

  public FlightAssist(IMyGridTerminalSystem gts, IMyProgrammableBlock pb) {
    GridTerminalSystem = gts;
    Me = pb;

    modules = new List<Module>();
    modules.Add(transPose = new TransPose(this, "TransPose"));
    modules.Add(hoverAssist = new HoverAssist(this, "HoverAssist"));
    modules.Add(vectorAssist = new VectorAssist(this, "VectorAssist"));
  }

  public void Run(string arguments) {
    string[] args = arguments.Split(' ');
    string moduleName = "";
    if (args.Length > 0)
      moduleName = args[0].ToLower();

    for (var i = 0; i < modules.Count; i++) {
      if (modules[i].name != "TransPose" || arguments == "")
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
  private string textPanelName = "Text panel 2"; // TODO: This does not belong in this module
  private string remoteControlName = "FA Remote";
  private int gyroCount = 2;

  private bool gyrosEnabled;

  public IMyRemoteControl remote;
  public IMyTextPanel screen;
  public List<IMyGyro> gyros;

  private Vector3D oldPosition;
  public Vector3D position;
  public Vector3D deltaPosition;

  public double speed;
  public double speedForward, speedRight, speedUp;

  private Matrix shipOrientation;
  private Matrix worldOrientation;
  private Vector3D forwardVec, rightVec, upVec;
  public double pitch, tilt;
  private Vector3D rotationVec;

  public Vector3D gravity;
  public bool inGravity;
  public bool switchingGravity;

  private double dt = 1000/60;

  public TransPose(FlightAssist fa, string n) : base(fa, n) {
    var list = new List<IMyTerminalBlock>();
    FlightAssist.GridTerminalSystem.GetBlocksOfType<IMyGyro>(list, x => x.CubeGrid == FlightAssist.Me.CubeGrid);
    gyros = list.ConvertAll(x => (IMyGyro)x);
    gyros = gyros.GetRange(0, gyroCount);

    remote = FlightAssist.GridTerminalSystem.GetBlockWithName(remoteControlName) as IMyRemoteControl;

    if (!String.IsNullOrEmpty(textPanelName))
      screen = FlightAssist.GridTerminalSystem.GetBlockWithName(textPanelName) as IMyTextPanel;

    remote.Orientation.GetMatrix(out shipOrientation);
  }

  override public void Tick() {
    CalcVelocity();
    CalcOrientation();
    CalcSpeedComponents();

    if (gyrosEnabled)
      SetGyroRpm();

    FlightAssist.transPose.screen.WritePublicText("Pitch: " + pitch +
                                                  "\nTilt: " + tilt +
                                                  "\nSpeed Forward: " + speedForward +
                                                  "\nSpeed Right: " + speedRight +
                                                  "\nSpeed Up: " + speedUp);
  }

  override public void ProcessCommand(string[] args) {
    if (args == null)
      return;

    var command = args[1].ToLower();

    if (command == "togglegyros") {
      ToggleGyros(!gyrosEnabled);
    }
  }

  private void CalcVelocity() {
    position = remote.GetPosition();
    deltaPosition = position - oldPosition;
    oldPosition = position;
    speed = deltaPosition.Length() / dt * 1000;
    deltaPosition.Normalize();
  }

  private void CalcOrientation() {
    gravity = -Vector3D.Normalize(remote.GetNaturalGravity());
    switchingGravity = inGravity;
    inGravity = !double.IsNaN(gravity.GetDim(0));
    switchingGravity = (inGravity != switchingGravity);

    worldOrientation = remote.WorldMatrix;
    SetOrientationVectors(inGravity ? GravityMainThrust : SpaceMainThrust);

    if (inGravity) {
      pitch = Math.Asin(Vector3D.Dot(gravity, forwardVec)) * (180/3.14159);
      tilt = Math.Asin(Vector3D.Dot(gravity, rightVec)) * (180/3.14159);
    } else {
      pitch = Math.Asin(Vector3D.Dot(deltaPosition, forwardVec)) * (180/3.14159);
      tilt = Math.Asin(Vector3D.Dot(deltaPosition, rightVec)) * (180/3.14159);
    }
  }

  private void CalcSpeedComponents() {
    if (inGravity) {
      speedForward = Vector3D.Dot(deltaPosition, Vector3D.Cross(gravity, rightVec)) * speed;
      speedRight = Vector3D.Dot(deltaPosition, -Vector3D.Cross(gravity, forwardVec)) * speed;
      speedUp = Vector3D.Dot(deltaPosition, gravity) * speed;
    } else {
      speedForward = Vector3D.Dot(deltaPosition, upVec) * speed;
      speedRight = Vector3D.Dot(deltaPosition, rightVec) * speed;
      speedUp = Vector3D.Dot(deltaPosition, forwardVec) * speed;
    }
  }

  private float ClampMinRpm(float val) {
    var minRpm = 0.015f;
    if (val > -minRpm && val < minRpm)
      return (float)(Math.Sign(val) * minRpm);
    else
      return val;
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
    for (int i = 0; i < gyros.Count; i++) {
      gyros[i].SetValueFloat("Pitch", 0f);
      gyros[i].SetValueFloat("Yaw", 0f);
      gyros[i].SetValueFloat("Roll", 0f);
      gyros[i].SetValueBool("Override", gyrosEnabled);
    }
  }

  public void ApproachTargetOrientation(double targetPitch, double targetTilt) {
    var max = gyros[0].GetMaximum<float>("Pitch");
    var pitchRate = ClampMinRpm((float)(max * (targetPitch - pitch) / 90));
    var tiltRate = ClampMinRpm((float)(max * (targetTilt - tilt) / 90));

    // When in space, prograde and retrograde are both (0, 0) so when flipping
    // to come to a stop, force a high turn rate at the beginning.
    if (!inGravity && speedForward > 0) {
      pitchRate = max - pitchRate;
      tiltRate = max - tiltRate;
    }

    rotationVec = inGravity ? new Vector3D(pitchRate, 0, tiltRate) : new Vector3D(pitchRate, -tiltRate, 0);
    rotationVec = Vector3.Transform(rotationVec, shipOrientation);
  }

  private void SetGyroRpm() {
    for (int i = 0; i < gyros.Count; i++) {
      var g = gyros[i];

      // Adjust rotation for the gyro's local orientation
      Matrix localOrientation;
      g.Orientation.GetMatrix(out localOrientation);
      var localRot = Vector3D.Transform(rotationVec, MatrixD.Transpose(localOrientation));

      g.SetValueFloat("Pitch", (float)localRot.GetDim(0));
      g.SetValueFloat("Yaw", (float)-localRot.GetDim(1));
      g.SetValueFloat("Roll", (float)-localRot.GetDim(2));
    }
  }
}

class VectorAssist : Module {
  private double angleThreshold = 0.3;
  private double speedThreshold = 0.3;

  private bool brakingEnabled;

  public VectorAssist(FlightAssist fa, string n) : base(fa, n) {}

  override public void Tick() {
    // Auto toggle off when entering space
    if (!FlightAssist.transPose.inGravity && FlightAssist.transPose.switchingGravity) {
      FlightAssist.transPose.ToggleGyros(false);
      brakingEnabled = false;
    }

    if (brakingEnabled)
      SpaceBrake();
  }

  override public void ProcessCommand(string[] args) {
    if (args == null)
      return;

    var command = args[1].ToLower();

    // Space only commands
    if (!FlightAssist.transPose.inGravity) {
      if (command == "brake") {
        brakingEnabled = !brakingEnabled;
        FlightAssist.transPose.ToggleGyros(brakingEnabled);
        if (FlightAssist.transPose.remote.DampenersOverride)
          FlightAssist.transPose.remote.GetActionWithName("DampenersOverride").Apply(FlightAssist.transPose.remote);
      }
    }
  }

  private void SpaceBrake() {
    if (FlightAssist.transPose.inGravity)
      return;

    // Stop when velocity is nearly 0
    if (FlightAssist.transPose.speed < speedThreshold) {
      brakingEnabled = false;
      FlightAssist.transPose.ToggleGyros(false);
      return;
    }

    // Activate dampeners when on target, if they aren't already on. Otherwise disable them.
    if (!FlightAssist.transPose.remote.DampenersOverride && 
        FlightAssist.transPose.speedForward < 0 && 
        Math.Abs(FlightAssist.transPose.pitch) < angleThreshold && 
        Math.Abs(FlightAssist.transPose.tilt) < angleThreshold) {
      FlightAssist.transPose.remote.GetActionWithName("DampenersOverride").Apply(FlightAssist.transPose.remote);
    }

    // Approach retrograde orientation
    FlightAssist.transPose.ApproachTargetOrientation(0, 0);
  }
}

class HoverAssist : Module {
  private double maxPitch = 45;
  private double maxRoll = 45;
  private int gyroResponsiveness = 16; // Larger = more gradual angle drop
  private bool alwaysEnabledInGravity = false;

  private bool hoverEnabled;
  private string mode;

  private double desiredPitch, desiredRoll;
  private float setSpeed;

  public HoverAssist(FlightAssist fa, string n) : base(fa, n) {}

  override public void Tick() {
    if (!FlightAssist.transPose.inGravity) {
      hoverEnabled = false;
      return;
    } else if (alwaysEnabledInGravity && hoverEnabled == false) {
      hoverEnabled = true;
    }

    if (hoverEnabled)
      ExecuteManeuver();
  }

  override public void ProcessCommand(string[] args) {
    if (args == null)
      return;

    mode = args[1].ToLower();

    // Gravity only commands
    if (FlightAssist.transPose.inGravity) {
      switch (mode) {
        case "toggle":
          hoverEnabled = !hoverEnabled;
          FlightAssist.transPose.ToggleGyros(hoverEnabled);
          break;

        case "cruise":
          setSpeed = args[2] != null ? Int32.Parse(args[2]) : 0;
          break;
      }
    }
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
        desiredRoll = FlightAssist.transPose.tilt;
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

    FlightAssist.transPose.ApproachTargetOrientation(desiredPitch, desiredRoll);
  }
}
