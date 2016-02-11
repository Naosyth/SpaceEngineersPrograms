// --------------------------------------------------
// ---------- Flight Assist -------------------------
// --------------------------------------------------
// Version 3.1
//
// Computer assisted flight
// Control your ship without thrusters in every direction
// 
// Author: Naosyth 
// naosyth@gmail.com 

// TODO
// Add error capturing and reporting
// Clean up horizon code
// Clean up printing in modules
// Allow targeting prograde or retrograde via vectorAssist
// Refactor state code if necessary
// Keep looking for blocks if required blocks aren't found or are lost

// --------------------------------------------------
// ---------- Configuration -------------------------
// --------------------------------------------------
// ----- Block Names -----
//   Name of key components on the ship required by this script
//   Screen is optional. If no screen is found, the script will sill work.
const string RemoteControlBlockName = "FA Remote";
const string TextPanelBlockName = "FA Screen";

// ----- Main Thrust Orientation -----
// Configure which side of the ship has the main thrusters for flight in space.
// Accepted values are 'forward', 'backward', 'right', 'left', 'up', and 'down'
const string SpaceMainThrust = "backward";

// ----- Gyro Configuration -----
// GyroCount - Number of gyros to use for orienting the ship
const int GyroCount = 1;

// ----- Hover Assist Configuration -----
// If set to true, the hover assist module will always be enabled while in gravity
const bool AlwaysEnabledInGravity = false;

// GyroResponsiveness - Adjust gyro responce curve. High = smooth but slower response.
const int GyroResponsiveness = 8;

// GyroVelocityScale - Scales gyro RPM. If you are overcorrecting, try lowering this.
const double GyroVelocityScale = 0.8;

// MaxPitch / MaxRoll - Maximum angle reached by the HoverAssist module when stopping the vehicle
const double MaxPitch = 45;
const double MaxRoll = 45;

// ----- Printer Module Configuration -----
// The number of ticks that must pass inbetween text panel screen re-draws
const int ScreenDrawInterval = 5;

// --------------------------------------------------
// ---------- Program -------------------------------
// --------------------------------------------------
const double HalfPi = Math.PI / 2;
const double RadToDeg = 180 / Math.PI;
const double DegToRad = Math.PI / 180;

bool initialized = false;
FlightAssist fa;

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
    modules.Add(transPose = new TransPose("TransPose"));
    modules.Add(hoverAssist = new HoverAssist("HoverAssist"));
    modules.Add(vectorAssist = new VectorAssist("VectorAssist"));
    modules.Add(printer = new Printer("Printer"));
  }

  public void Run(string arguments) {
    string[] args = arguments.Split(' ');
    for (var i = 0; i < modules.Count; i++)
      modules[i].Run(args);
  }
}

abstract class Module : Program {
  public string name;
  private string printBuffer;

  public Module(string n) {
    name = n;
  } 

  virtual public void Run(string[] args) {
    printBuffer = "";
    if (args[0] == name)
      ProcessCommand(args);
    else
      Tick();
  }

  abstract public void Tick();
  virtual public void ProcessCommand(string[] args) {}
  public string GetPrintString() { return printBuffer; }
  protected void PrintLine(string line) { printBuffer += "\n" + line; }
}

class Printer : Module {
  private int redrawInterval = ScreenDrawInterval;
  private string textPanelName = TextPanelBlockName;
  public IMyTextPanel screen;

  private List<Module> printableModules;
  private int numModules;
  private int displayedModule;
  private int ticks;

  public Printer(string name) : base(name) {
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

    ticks += 1;
    if (ticks % redrawInterval != 0)
      return;

    var module = printableModules[displayedModule];
    screen.WritePublicText("");
    screen.WritePublicText("Flight Assist - Module [" + module.name + "]");
    screen.WritePublicText("\n" + module.GetPrintString(), true);
  }
}

class TransPose : Module {
  private string remoteControlName = RemoteControlBlockName;
  private int gyroCount = GyroCount;
  private double gyroVelocityScale = GyroVelocityScale;
  private double minGyroRpmScale = 0.015;

  private bool gyrosEnabled;
  public IMyRemoteControl remote;
  public List<IMyGyro> gyros;

  private Vector3D oldPosition;
  public Vector3D position;
  public Vector3D deltaPosition;
  public double speed;
  public double localSpeedForward, localSpeedRight, localSpeedUp;

  public Matrix shipOrientation;
  public Matrix worldOrientation;

  public Vector3D gravity;
  public bool inGravity;
  public bool switchingGravity;

  private Vector3D reference, target;
  public double angle;
  private double dt = 1000/60;

  public TransPose(string name) : base(name) {
    var list = new List<IMyTerminalBlock>();
    FlightAssist.GridTerminalSystem.GetBlocksOfType<IMyGyro>(list, x => x.CubeGrid == FlightAssist.Me.CubeGrid);
    gyros = list.ConvertAll(x => (IMyGyro)x);
    gyros = gyros.GetRange(0, gyroCount);

    remote = FlightAssist.GridTerminalSystem.GetBlockWithName(remoteControlName) as IMyRemoteControl;
    remote.Orientation.GetMatrix(out shipOrientation);
  }

  override public void Tick() {
    CalcVelocity();
    CalcGravity();
    worldOrientation = remote.WorldMatrix;

    if (gyrosEnabled)
      SetGyroRpm();

    PrintLine("----- Velocity ----------------------------------------" +
              "\nTotal: " + String.Format("{0:000}", speed) + " m/s" +
              "\n  F/B: " + String.Format("{0:000}", localSpeedForward) +
              "\n  R/L: " + String.Format("{0:000}", localSpeedRight) +
              "\n  U/D: " + String.Format("{0:000}", localSpeedUp));
  }

  override public void ProcessCommand(string[] args) {
    var command = args[1].ToLower();
    if (command == "togglegyros")
      ToggleGyros(!gyrosEnabled);
  }

  private void CalcVelocity() {
    position = remote.GetPosition();
    deltaPosition = position - oldPosition;
    oldPosition = position;
    speed = deltaPosition.Length() / dt * 1000;
    deltaPosition.Normalize();

    localSpeedUp = NotNan(Vector3D.Dot(deltaPosition, worldOrientation.Up) * speed);
    localSpeedRight = NotNan(Vector3D.Dot(deltaPosition, worldOrientation.Right) * speed);
    localSpeedForward = NotNan(Vector3D.Dot(deltaPosition, worldOrientation.Forward) * speed);
  }

  private void CalcGravity() {
    gravity = -Vector3D.Normalize(remote.GetNaturalGravity());
    switchingGravity = inGravity;
    inGravity = !double.IsNaN(gravity.GetDim(0));
    switchingGravity = (inGravity != switchingGravity);
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

  public void SetTargetOrientation(Vector3D setReference, Vector3D setTarget) {
    reference = setReference;
    target = setTarget; 
  }

  private void SetGyroRpm() {
    for (int i = 0; i < gyros.Count; i++) {
      var g = gyros[i];

      Matrix localOrientation;
      g.Orientation.GetMatrix(out localOrientation);
      var localReference = Vector3D.Transform(reference, MatrixD.Transpose(localOrientation));
      var localTarget = Vector3D.Transform(target, MatrixD.Transpose(g.WorldMatrix.GetOrientation()));

      var axis = Vector3D.Cross(localReference, localTarget);
      angle = axis.Length();
      angle = Math.Atan2(angle, Math.Sqrt(Math.Max(0.0, 1.0 - angle * angle)));
      axis.Normalize();
      axis *= Math.Max(minGyroRpmScale, g.GetMaximum<float>("Pitch") * (angle / Math.PI) * gyroVelocityScale);

      g.SetValueFloat("Pitch", (float)axis.GetDim(0));
      g.SetValueFloat("Yaw", (float)-axis.GetDim(1));
      g.SetValueFloat("Roll", (float)-axis.GetDim(2));
    }
  }
}

class VectorAssist : Module {
  private double angleThreshold = 0.03;
  private double speedThreshold = 0.3;

  private bool brakingEnabled;
  private double startSpeed;

  public VectorAssist(string name) : base(name) {}

  override public void Tick() {
    // Auto toggle off when entering space
    if (FlightAssist.transPose.switchingGravity && !FlightAssist.transPose.inGravity) {
      FlightAssist.transPose.ToggleGyros(false);
      brakingEnabled = false;
    }

    if (brakingEnabled)
      SpaceBrake();

    PrintLine(BuildVisual());
  }

  override public void ProcessCommand(string[] args) {
    var command = args[1].ToLower();
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

  // TODO: Clean this up
  private string BuildVisual() {
    int height = 13;
    int width = 27;

    int yCenter = height/2;
    int xCenter = width/2;

    double pitch, tilt;
    if (FlightAssist.transPose.inGravity) {
      pitch = NotNan(Math.Acos(Vector3D.Dot(FlightAssist.transPose.worldOrientation.Forward, FlightAssist.transPose.gravity)) * RadToDeg);
      tilt = NotNan(Math.Acos(Vector3D.Dot(FlightAssist.transPose.worldOrientation.Right, FlightAssist.transPose.gravity)) * RadToDeg);
      pitch = height - Math.Floor((pitch / 180) * height) - 1;
      tilt = Math.Floor(Vector3D.Dot(FlightAssist.transPose.worldOrientation.Right, FlightAssist.transPose.deltaPosition) * xCenter) + xCenter;
      if (FlightAssist.transPose.localSpeedForward < 0)
        tilt = width - tilt;
    } else {
      pitch = -Math.Floor((FlightAssist.transPose.localSpeedUp / FlightAssist.transPose.speed) * yCenter) + yCenter;
      tilt = Math.Floor((FlightAssist.transPose.localSpeedRight / FlightAssist.transPose.speed) * xCenter) + xCenter;
    }

    double roll, horizon, horizonPoint;
    roll = Math.Asin(Vector3.Dot(FlightAssist.transPose.gravity, FlightAssist.transPose.worldOrientation.Right)) * RadToDeg;
    horizonPoint = pitch;
    horizon = pitch;

    string output = "";
    for (var y = 0; y < height; y++) {
      output += "       |";
      for (var x = 0; x < width; x++) {
        var upsideDown = Vector3D.Dot(FlightAssist.transPose.worldOrientation.Up, FlightAssist.transPose.gravity) < 0;

        if (FlightAssist.transPose.inGravity) {
          if (EqualWithMargin(roll, 0, 0.01)) {
            horizonPoint = horizon;
          } else {
            horizonPoint = Math.Floor((height * 2 * roll / 90)) * (x - xCenter) / (width/2) + horizon;
            if (upsideDown)
              horizonPoint = height - horizonPoint;
          }
        }

        if (FlightAssist.transPose.inGravity && (x == tilt && y == 0))
          output += ".!";
        else if (!FlightAssist.transPose.inGravity && x == tilt && y == pitch)
          output += FlightAssist.transPose.localSpeedForward < 0 ? "~" : "+";
        else if (x == xCenter && y == yCenter)
          output += "  ";
        else if (x == xCenter-1 && y == yCenter)
          output += "<";
        else if (x == xCenter+1 && y == yCenter)
          output += ">";
        else if (FlightAssist.transPose.inGravity && !upsideDown && y > horizonPoint)
          output += "=";
        else if (FlightAssist.transPose.inGravity && upsideDown && y < horizonPoint)
          output += "=";
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

    // Activate dampeners when on target, if they aren't already on. Otherwise disable them.
    if (!FlightAssist.transPose.remote.DampenersOverride && EqualWithMargin(FlightAssist.transPose.angle, 0, angleThreshold)) {
      FlightAssist.transPose.remote.GetActionWithName("DampenersOverride").Apply(FlightAssist.transPose.remote);
    }

    // Approach retrograde orientation
    FlightAssist.transPose.SetTargetOrientation(FlightAssist.transPose.shipOrientation.Down, FlightAssist.transPose.deltaPosition);
  }
}

class HoverAssist : Module {
  private bool alwaysEnabledInGravity = AlwaysEnabledInGravity;
  private int gyroResponsiveness = GyroResponsiveness;
  private double maxPitch = MaxPitch;
  private double maxRoll = MaxRoll;

  private bool hoverEnabled;
  private string mode;
  private float setSpeed;

  private double pitch, roll;
  private double worldSpeedForward, worldSpeedRight, worldSpeedUp;

  public HoverAssist(string name) : base(name) {
    alwaysEnabledInGravity = AlwaysEnabledInGravity;
    gyroResponsiveness = GyroResponsiveness;

    mode = "Hover";
    hoverEnabled = FlightAssist.transPose.gyros[0].GyroOverride;
    FlightAssist.transPose.ToggleGyros(hoverEnabled);
  }

  override public void Tick() {
    if (!FlightAssist.transPose.inGravity) {
      hoverEnabled = false;
      return;
    } else if (alwaysEnabledInGravity && hoverEnabled == false) {
      hoverEnabled = true;
      FlightAssist.transPose.ToggleGyros(true);
    }

    CalcWorldSpeed();
    CalcPitchAndRoll();
    if (hoverEnabled)
      ExecuteManeuver();

    string output = "----- Status -------------------------------------------" +
                    "\nHover State: " + (hoverEnabled ? "ENABLED" : "DISABLED") +
                    "\nHover Mode: " + mode.ToUpper();
    if (FlightAssist.transPose.inGravity) {
      output += "\n\n----- Velocity ----------------------------------------" +
              "\nTotal: " + String.Format("{0:000}", FlightAssist.transPose.speed) + " m/s" +
              "\n  F/B: " + String.Format("{0:000}", worldSpeedForward) +
              "\n  R/L: " + String.Format("{0:000}", worldSpeedRight) +
              "\n  U/D: " + String.Format("{0:000}", worldSpeedUp) +
              "\n\n----- Orientation ----------------------------------------" +
              "\nPitch: " + String.Format("{0:00}", pitch) + "°" +
              " | Roll: " + String.Format("{0:00}", roll) + "°";
    }

    PrintLine(output);
  }

  override public void ProcessCommand(string[] args) {
    var command = args[1];
    if (FlightAssist.transPose.inGravity) {
      switch (command.ToLower()) {
        case "toggle":
          hoverEnabled = !hoverEnabled;
          FlightAssist.transPose.ToggleGyros(hoverEnabled);
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

  private void CalcWorldSpeed() {
    worldSpeedForward = NotNan(Vector3D.Dot(FlightAssist.transPose.deltaPosition, Vector3D.Cross(FlightAssist.transPose.gravity, FlightAssist.transPose.worldOrientation.Right)) * FlightAssist.transPose.speed);
    worldSpeedRight = NotNan(Vector3D.Dot(FlightAssist.transPose.deltaPosition, Vector3D.Cross(FlightAssist.transPose.gravity, FlightAssist.transPose.worldOrientation.Forward)) * FlightAssist.transPose.speed);
    worldSpeedUp = NotNan(Vector3D.Dot(FlightAssist.transPose.deltaPosition, FlightAssist.transPose.gravity));
  }

  private void CalcPitchAndRoll() {
    pitch = NotNan(Math.Acos(Vector3D.Dot(FlightAssist.transPose.worldOrientation.Forward, FlightAssist.transPose.gravity)) * RadToDeg);
    roll = NotNan(Math.Acos(Vector3D.Dot(FlightAssist.transPose.worldOrientation.Right, FlightAssist.transPose.gravity)) * RadToDeg);
  }

  private void ExecuteManeuver() {
    double desiredPitch, desiredRoll;
    switch (mode.ToLower()) {
      case "glide":
        desiredPitch = 0;
        desiredRoll = Math.Atan(worldSpeedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      case "freeglide":
        desiredPitch = 0;
        desiredRoll = 0;
        break;

      case "pitch":
        desiredPitch = Math.Atan(worldSpeedForward / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = (roll - 90);
        break;

      case "roll":
        desiredPitch = -(pitch - 90);
        desiredRoll = Math.Atan(worldSpeedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      case "cruise":
        desiredPitch = Math.Atan((worldSpeedForward - setSpeed) / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = Math.Atan(worldSpeedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;

      default: // Stationary Hover
        desiredPitch = Math.Atan(worldSpeedForward / gyroResponsiveness) / HalfPi * maxPitch;
        desiredRoll = Math.Atan(worldSpeedRight / gyroResponsiveness) / HalfPi * maxRoll;
        break;
    }

    var quatPitch = Quaternion.CreateFromAxisAngle(FlightAssist.transPose.shipOrientation.Left, (float)(desiredPitch * DegToRad));
    var quatRoll = Quaternion.CreateFromAxisAngle(FlightAssist.transPose.shipOrientation.Backward, (float)(desiredRoll * DegToRad));
    var reference = Vector3D.Transform(FlightAssist.transPose.shipOrientation.Down, quatPitch * quatRoll);
    FlightAssist.transPose.SetTargetOrientation(reference, FlightAssist.transPose.remote.GetNaturalGravity());
  }
}

// --------------------------------------------------
// ---------- Helper Functions ----------------------
// --------------------------------------------------

bool EqualWithMargin(double value, double target, double margin) {
  return value > target - margin && value < target + margin;
}

Vector3D GetThrustVector(string direction) {
  switch (direction.ToLower()) {
    case "down": return FlightAssist.transPose.shipOrientation.Down;
    case "up": return FlightAssist.transPose.shipOrientation.Up;
    case "forward": return FlightAssist.transPose.shipOrientation.Forward;
    case "backward": return FlightAssist.transPose.shipOrientation.Backward;
    case "right": return FlightAssist.transPose.shipOrientation.Right;
    case "left": return FlightAssist.transPose.shipOrientation.Left;
    default: return new Vector3D(0,0,0);
  }
}

double NotNan(double val) {
  if (double.IsNaN(val)) return 0;
  return val;
}
