// TODO
// * Display other useful info such as dampener status
// * Add text panel draw rate limiter
// * Make configuration easier
// * Fix FlightAssist -> flightAssist

// To do rear in atmo:
// * Remove "90 - " in desiredPitch
// * Change worldSpeedRight to use upVec

// To do bottom in space:
// * change desired to (90, 90)

// Configure which side of the ship has the main thrusters
// for flight in both gravity and space.
// Accepted values are 'bottom' and 'rear'
static string GravityMainThrust = "rear";
static string SpaceMainThrust = "rear";

bool initialized = false;

FlightAssist fa;

const double HalfPi = Math.PI / 2;
const double RadToDeg = 180 / Math.PI;

void Main(string arguments) {
  if (!initialized) {
    fa = new FlightAssist(GridTerminalSystem, Me);

    initialized = true;
  }

  fa.Run(arguments);
}

class FlightAssist {
  private List<Module> modules;

  public static IMyProgrammableBlock Me;
  public static IMyGridTerminalSystem GridTerminalSystem;

  public static TransPose transPose;
  public static Printer printer;
  public static HoverAssist hoverAssist;
  public static VectorAssist vectorAssist;

  public FlightAssist(IMyGridTerminalSystem gts, IMyProgrammableBlock pb) {
    GridTerminalSystem = gts;
    Me = pb;

    modules = new List<Module>();
    modules.Add(transPose = new TransPose(this, "TransPose"));
    modules.Add(hoverAssist = new HoverAssist(this, "HoverAssist"));
    modules.Add(vectorAssist = new VectorAssist(this, "VectorAssist"));
    modules.Add(printer = new Printer(this, "Printer"));
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

  virtual public string GetPrintString() { return ""; }
}

class TransPose : Module {
  private string remoteControlName = "FA Remote";
  private int gyroCount = 2;

  private bool gyrosEnabled;

  public IMyRemoteControl remote;
  public List<IMyGyro> gyros;

  private Vector3D oldPosition;
  public Vector3D position;
  public Vector3D deltaPosition;

  public double speed;
  public double worldSpeedForward, worldSpeedRight, worldSpeedUp;
  public double localSpeedForward, localSpeedRight, localSpeedUp;

  private Matrix shipOrientation;
  private Matrix worldOrientation;
  public Vector3D forwardVec, rightVec, upVec;
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
    remote.Orientation.GetMatrix(out shipOrientation);

    ToggleGyros(false);
  }

  override public void Tick() {
    CalcVelocity();
    CalcOrientation();
    CalcSpeedComponents();

    if (gyrosEnabled)
      SetGyroRpm();
  }

  override public void ProcessCommand(string[] args) {
    if (args == null)
      return;

    var command = args[1].ToLower();

    if (command == "togglegyros") {
      ToggleGyros(!gyrosEnabled);
    }
  }

  override public string GetPrintString() {
    return "----- Velocity ----------------------------------------" +
           "\nTotal: " + String.Format("{0:000}", speed) + " m/s" +
           "\n  F/B: " + String.Format("{0:000}", localSpeedForward) +
           "\n  R/L: " + String.Format("{0:000}", localSpeedRight) +
           "\n  U/D: " + String.Format("{0:000}", localSpeedUp) +
           "\n\n----- Orientation ----------------------------------------" +
           "\nPitch: " + String.Format("{0:00}", pitch) + "°" +
           " | Tilt: " + String.Format("{0:00}", tilt) + "°";
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
    forwardVec = worldOrientation.Forward;
    rightVec = worldOrientation.Right;
    upVec = worldOrientation.Up;

    if (inGravity) {
      pitch = Math.Acos(Vector3D.Dot(gravity, forwardVec)) * RadToDeg;
      tilt = Math.Acos(Vector3D.Dot(gravity, rightVec)) * RadToDeg;
      if (double.IsNaN(pitch)) pitch = 0;
      if (double.IsNaN(tilt)) tilt = 0;
    } else {
      if (double.IsNaN(deltaPosition.Length())) {
        pitch = 0;
        tilt = 0;
        return;
      }
      pitch = Math.Acos(Vector3D.Dot(deltaPosition, forwardVec)) * RadToDeg;

      if (Double.IsNaN(Math.Acos(Vector3D.Dot(deltaPosition, forwardVec))))
        pitch = localSpeedForward > 0 ? 0 : 180;

      tilt = Math.Acos(Vector3D.Dot(deltaPosition, rightVec)) * RadToDeg;
    }
  }

  private void CalcSpeedComponents() {
    if (double.IsNaN(deltaPosition.Length())) {
      worldSpeedForward = 0;
      worldSpeedRight = 0;
      worldSpeedUp = 0;
      localSpeedForward = 0;
      localSpeedRight = 0;
      localSpeedUp = 0;
      return;
    }

    if (inGravity) {
      worldSpeedForward = Vector3D.Dot(deltaPosition, Vector3D.Cross(gravity, rightVec)) * speed;
      if (GravityMainThrust == "bottom")
        worldSpeedRight = Vector3D.Dot(deltaPosition, -Vector3D.Cross(gravity, forwardVec)) * speed;
      else
        worldSpeedRight = -Vector3D.Dot(deltaPosition, -Vector3D.Cross(gravity, upVec)) * speed;
      worldSpeedUp = Vector3D.Dot(deltaPosition, gravity) * speed;
    } else {
      worldSpeedForward = 0;
      worldSpeedRight = 0;
      worldSpeedUp = 0;
    }

    if (SpaceMainThrust == "rear") {
      localSpeedForward = Vector3D.Dot(deltaPosition, forwardVec) * speed;
      localSpeedRight = Vector3D.Dot(deltaPosition, rightVec) * speed;
      localSpeedUp = Vector3D.Dot(deltaPosition, upVec) * speed;
    } else {
      localSpeedUp = -Vector3D.Dot(deltaPosition, forwardVec) * speed;
      localSpeedRight = Vector3D.Dot(deltaPosition, rightVec) * speed;
      localSpeedForward = Vector3D.Dot(deltaPosition, upVec) * speed;
    }
  }

  private double ClampMinRpm(double val) {
    var minRpm = 0.015f;
    if (val > -minRpm && val < minRpm)
      return (Math.Sign(val) * minRpm);
    else
      return val;
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
    double pitchRate, yawRate, rollRate;
    if (inGravity && GravityMainThrust == "rear")
      pitchRate = (max * (targetPitch - (pitch * -Math.Sign(Vector3D.Dot(upVec, gravity)))) / 180);
    else
      pitchRate = (max * (targetPitch - pitch) / 180);
    yawRate = (max * (targetTilt - tilt) / 180);
    rollRate = (gyros[0].GetMaximum<float>("Roll") * (targetTilt - tilt) / 180);

    if (!inGravity) {
      if (SpaceMainThrust == "bottom")
        pitchRate = Math.Abs(pitchRate);

      pitchRate *= -Math.Sign(localSpeedUp);
      rollRate *= Math.Sign(localSpeedForward);

      if (SpaceMainThrust == "bottom" && localSpeedForward > 0) {
        pitchRate = max - pitchRate;
        yawRate = max - yawRate;
        rollRate = gyros[0].GetMaximum<float>("Roll") - rollRate;
      }

      yawRate *= 1.0f - (float)(pitch / 90);
      rollRate *= (float)(pitch / 90);
    } else if (inGravity) {
      if (GravityMainThrust == "bottom") {
        pitchRate *= -Math.Sign(Vector3D.Dot(upVec, gravity));
      }

      yawRate *= 1.0 - pitch == 90 ? 1.0 : (Math.Abs(90 - pitch) / 90);
      rollRate *= pitch == 90 ? 1.0 : (Math.Abs(90 - pitch) / 90);
    }

    rotationVec = new Vector3D(ClampMinRpm(pitchRate), ClampMinRpm(-yawRate), ClampMinRpm(-rollRate));
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

class Printer : Module {
  private string textPanelName = "Text panel 2";
  public IMyTextPanel screen;

  private List<Module> printableModules;
  private int numModules;
  private int displayedModule;

  public Printer(FlightAssist fa, string n) : base(fa, n) {
    if (!String.IsNullOrEmpty(textPanelName))
      screen = FlightAssist.GridTerminalSystem.GetBlockWithName(textPanelName) as IMyTextPanel;

    printableModules = new List<Module>();
    printableModules.Add(FlightAssist.transPose);
    printableModules.Add(FlightAssist.hoverAssist);
    printableModules.Add(FlightAssist.vectorAssist);
    numModules = printableModules.Count;
  }

  override public void ProcessCommand(string[] args) {
    if (args == null)
      return;

    var command = args[1].ToLower();

    switch (command) {
      case "next":
        displayedModule = (displayedModule + 1) % numModules;
        break;
      case "previous":
        displayedModule = (displayedModule == 0 ? numModules - 1 : displayedModule - 1) % numModules;
        break;
    }
  }

  override public void Tick() {
    if (screen == null)
      return;

    var module = printableModules[displayedModule];
    screen.WritePublicText("");
    screen.WritePublicText("Flight Assist - Module [" + module.name + "]");
    screen.WritePublicText("\n" + module.GetPrintString(), true);
  }
}

class VectorAssist : Module {
  private double angleThreshold = 0.3;
  private double speedThreshold = 0.3;

  private bool brakingEnabled;
  private double startSpeed;

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
        startSpeed = FlightAssist.transPose.speed;
        FlightAssist.transPose.ToggleGyros(brakingEnabled);
        if (FlightAssist.transPose.remote.DampenersOverride)
          FlightAssist.transPose.remote.GetActionWithName("DampenersOverride").Apply(FlightAssist.transPose.remote);
      }
    }
  }

  override public string GetPrintString() {
    return BuildVisual();
  }

  private string BuildVisual() {
    int height = 13;
    int width = 27;

    int yCenter = height/2;
    int xCenter = width/2;

    double pitch, tilt;

    pitch = height - Math.Floor((FlightAssist.transPose.pitch / 180) * height);
    tilt = Math.Floor(Vector3D.Dot(FlightAssist.transPose.rightVec, FlightAssist.transPose.deltaPosition) * xCenter) + xCenter;

    if (FlightAssist.transPose.localSpeedForward < 0){
      pitch = height - pitch;
      tilt = width - tilt;
    }

    string output = "";
    for (var y = 0; y < height; y++) {
      output += "       |";
      for (var x = 0; x < width; x++) {
        if (x == tilt && y == pitch)
          output += FlightAssist.transPose.localSpeedForward < 0 ? "~" : "+";
        else if (x == xCenter && y == yCenter)
          output += "  ";
        else if (x == xCenter-1 && y == yCenter)
          output += "<";
        else if (x == xCenter+1 && y == yCenter)
          output += ">";
        else
          output += ". ";
      }
      output += "|\n";
    }

    if (brakingEnabled) {
      var percent = Math.Abs(FlightAssist.transPose.speed / startSpeed);
      output += "       |";
      for (var i = 0; i < width; i++) {
        if (i < width * percent)
          output += "=";
        else
          output += "~";
      }
      output += "|";
      output += "\n       |  Braking In Progress                       |";
    }

    return output;
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

    double desiredPitch, desiredTilt;
    if (SpaceMainThrust == "rear") {
      desiredPitch = 180;
      desiredTilt = 90;
    } else {
      desiredPitch = 90;
      desiredTilt = 90;
    }

    // Activate dampeners when on target, if they aren't already on. Otherwise disable them.
    if (!FlightAssist.transPose.remote.DampenersOverride && 
        FlightAssist.transPose.localSpeedForward < 0 && 
        EqualWithMargin(Math.Abs(FlightAssist.transPose.pitch), desiredPitch, angleThreshold) && 
        EqualWithMargin(Math.Abs(FlightAssist.transPose.tilt), desiredTilt, angleThreshold)) {
      FlightAssist.transPose.remote.GetActionWithName("DampenersOverride").Apply(FlightAssist.transPose.remote);
    }

    // Approach retrograde orientation
    FlightAssist.transPose.ApproachTargetOrientation(desiredPitch, desiredTilt);
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

  public HoverAssist(FlightAssist fa, string n) : base(fa, n) {
    mode = "Hover";
  }

  override public void Tick() {
    if (!FlightAssist.transPose.inGravity) {
      hoverEnabled = false;
      return;
    } else if (alwaysEnabledInGravity && hoverEnabled == false) {
      hoverEnabled = true;
      FlightAssist.transPose.ToggleGyros(true);
    }

    if (hoverEnabled)
      ExecuteManeuver();
  }

  override public void ProcessCommand(string[] args) {
    if (args == null)
      return;

    var command = args[1];

    // Gravity only commands
    if (FlightAssist.transPose.inGravity) {
      switch (command.ToLower()) {
        case "toggle":
          if (FlightAssist.transPose.inGravity) {
            hoverEnabled = !hoverEnabled;
            FlightAssist.transPose.ToggleGyros(hoverEnabled);
          }
          break;

        case "cruise":
          setSpeed = args[2] != null ? Int32.Parse(args[2]) : 0;
          break;

        default:
          mode = command;
          break;
      }
    }
  }

  override public string GetPrintString() {
    string output = "----- Status -------------------------------------------" +
                    "\nHover State: " + (hoverEnabled ? "ENABLED" : "DISABLED") +
                    "\nHover Mode: " + mode.ToUpper();
    if (FlightAssist.transPose.inGravity) {
      output += "\n\n----- Velocity ----------------------------------------" +
              "\nTotal: " + String.Format("{0:000}", FlightAssist.transPose.speed) + " m/s" +
              "\n  F/B: " + String.Format("{0:000}", FlightAssist.transPose.worldSpeedForward) +
              "\n  R/L: " + String.Format("{0:000}", FlightAssist.transPose.worldSpeedRight) +
              "\n  U/D: " + String.Format("{0:000}", FlightAssist.transPose.worldSpeedUp) +
              "\n\n----- Orientation ----------------------------------------" +
              "\nPitch: " + String.Format("{0:00}", FlightAssist.transPose.pitch) + "°" +
              " | Roll: " + String.Format("{0:00}", FlightAssist.transPose.tilt) + "°";
    }
    return output;
  }

  private void ExecuteManeuver() {
    switch (mode.ToLower()) {
      case "glide":
        desiredPitch = 0;
        desiredRoll = Math.Atan(FlightAssist.transPose.worldSpeedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      case "freeglide":
        desiredPitch = 0;
        desiredRoll = 0;
        break;

      case "pitch":
        desiredPitch = Math.Atan(FlightAssist.transPose.worldSpeedForward / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = FlightAssist.transPose.tilt;
        break;

      case "roll":
        desiredPitch = FlightAssist.transPose.pitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.worldSpeedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      case "cruise":
        desiredPitch = Math.Atan((FlightAssist.transPose.worldSpeedForward - setSpeed) / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.worldSpeedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      default: // Stationary Hover
        desiredPitch = Math.Atan(FlightAssist.transPose.worldSpeedForward / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.worldSpeedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;
    }

    if (GravityMainThrust == "bottom")
      desiredPitch = 90 - desiredPitch;
    desiredRoll = 90 - desiredRoll;

    FlightAssist.transPose.ApproachTargetOrientation(desiredPitch, desiredRoll);
  }
}

static bool EqualWithMargin(double value, double target, double margin) {
  return value > target - margin && value < target + margin;
}
