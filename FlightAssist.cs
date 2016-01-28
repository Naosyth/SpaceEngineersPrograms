// TODO
// Enable/Disable Vector Control
// Move maxpitch/etc into appropriate modules
// DRY up code
// Allow configuring which side of ship has main thrusters in gravity and atmosphere

bool initialized = false;

FlightAssist fa;

static double MaxPitch = 67.5;
static double MaxRoll = 67.5;
static int GyroResponsiveness = 8; // Larger = more gradual angle drop
static double minRPM = 0.015; // Min RPM setting for gyros. Values that are too low behave weird. You shouldn't need to adjust this.

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

  public static BlockManager blockManager;
  public static TransPose transPose;
  public static AutoHover autoHover;
  public static VectorControl vectorControl;

  public FlightAssist(IMyGridTerminalSystem gts, IMyProgrammableBlock pb) {
    GridTerminalSystem = gts;
    Me = pb;

    modules = new List<Module>();
    modules.Add(blockManager = new BlockManager(this, "BlockManager"));
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

class BlockManager : Module {
  private string GyroName = "FA Gyro";
  private string TextPanelName = "Text panel 2";
  private string RemoteControlName = "FA Remote";
  private int GyroCount = 1;

  public IMyRemoteControl remote;
  public IMyTextPanel screen;
  public IMyGyro gyro;
  private List<IMyGyro> gyros;

  public BlockManager(FlightAssist fa, string n) : base(fa, n) {
    gyro = FlightAssist.GridTerminalSystem.GetBlockWithName(GyroName) as IMyGyro;

    var list = new List<IMyTerminalBlock>();
    FlightAssist.GridTerminalSystem.GetBlocksOfType<IMyGyro>(list, x => x.CubeGrid == FlightAssist.Me.CubeGrid && x != gyro);
    gyros = list.ConvertAll(x => (IMyGyro)x);
    gyros.Insert(0, gyro);
    gyros = gyros.GetRange(0, GyroCount);

    remote = FlightAssist.GridTerminalSystem.GetBlockWithName(RemoteControlName) as IMyRemoteControl;

    if (!String.IsNullOrEmpty(TextPanelName))
      screen = FlightAssist.GridTerminalSystem.GetBlockWithName(TextPanelName) as IMyTextPanel;
  }

  override public void Tick() {}

  override public void ProcessCommand(string[] args) {}

  public void ToggleGyros(bool state) {
    for (int i = 0; i < gyros.Count; i++)
      gyros[i].SetValueBool("Override", state);
  }

  public void SetGyros(Vector3 rotationVector, bool overrideEnabled) {
    for (int i = 0; i < gyros.Count; i++) {
      var g = gyros[i];

      // Adjust rotation for the gyro's local orientation
      Matrix localOrientation;
      g.Orientation.GetMatrix(out localOrientation);
      var localRot = Vector3.Transform(rotationVector, MatrixD.Transpose(localOrientation));

      g.SetValueFloat("Pitch", (float)localRot.GetDim(0));
      g.SetValueFloat("Yaw", (float)-localRot.GetDim(1));
      g.SetValueFloat("Roll", (float)-localRot.GetDim(2));
      g.SetValueBool("Override", gyro.GyroOverride);
    }
  }
}

class TransPose : Module {
  private Vector3D oldPosition;
  public Vector3D position;
  public Vector3D deltaPosition;

  public Vector3 velocity;
  public double speed;
  public double speedForward, speedRight, speedUp;

  private Matrix orientation;
  public double roll, pitch, yaw;

  private Vector3D forwardVec, rightVec, upVec;

  public Vector3D gravity;
  public bool inGravity;
  public bool switchingGravity;

  private double dt = 1000/60;

  public TransPose(FlightAssist fa, string n) : base(fa, n) {}

  override public void Tick() {
    CalcVelocity();
    CalcOrientation();
    CalcSpeedComponents();
  }

  override public void ProcessCommand(string[] args) {}

  private void CalcVelocity() {
    position = FlightAssist.blockManager.remote.GetPosition();
    deltaPosition = position - oldPosition;
    oldPosition = position;
    speed = deltaPosition.Length() / dt * 1000;
  }

  private void CalcOrientation() {
    gravity = -Vector3D.Normalize(FlightAssist.blockManager.remote.GetNaturalGravity());
    switchingGravity = inGravity;
    inGravity = !double.IsNaN(gravity.GetDim(0));
    switchingGravity = inGravity != switchingGravity;

    orientation = FlightAssist.blockManager.remote.WorldMatrix;
    if (inGravity) {
      forwardVec = orientation.Forward;
      rightVec = orientation.Right;
      upVec = orientation.Up;
    } else {
      forwardVec = orientation.Up;
      rightVec = orientation.Right;
      upVec = orientation.Forward;
    }

    if (inGravity) {
      pitch = Vector3D.Dot(gravity, forwardVec) / (gravity.Length() * forwardVec.Length()) * 90;
      roll = Vector3D.Dot(gravity, rightVec) / (gravity.Length() * rightVec.Length()) * 90;
    } else {
      pitch = Vector3D.Dot(deltaPosition, forwardVec) / (deltaPosition.Length() * forwardVec.Length()) * 90;
      roll = Vector3D.Dot(deltaPosition, rightVec) / (deltaPosition.Length() * rightVec.Length()) * 90;
      if (speed == 0) {
        roll = 0;
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
}

class AutoHover : Module {
  private string mode;

  private double desiredPitch, desiredRoll;
  private float pitchRate, rollRate;
  private float setSpeed;

  public AutoHover(FlightAssist fa, string n) : base(fa, n) {
    mode = "Hover";
  }

  override public void Tick() {
    FlightAssist.blockManager.screen.WritePublicText("Mode: " + mode);
    ExecuteManeuver();
  }

  override public void ProcessCommand(string[] args) {
    if (args == null) return;
    mode = args[1];
  }

  private void ExecuteManeuver() {
    switch (mode.ToLower()) {
      case "glide":
        desiredPitch = 0;
        desiredRoll = Math.Atan(FlightAssist.transPose.speedRight / GyroResponsiveness) / HalfPi * MaxRoll;
        break;

      case "freeglide":
        desiredPitch = 0;
        desiredRoll = 0;
        break;

      case "pitch":
        desiredPitch = Math.Atan(FlightAssist.transPose.speedForward / GyroResponsiveness) / HalfPi * MaxPitch;
        desiredRoll = FlightAssist.transPose.roll;
        break;

      case "roll":
        desiredPitch = FlightAssist.transPose.pitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.speedRight / GyroResponsiveness) / HalfPi * MaxRoll;
        break;

      case "cruise":
        desiredPitch = Math.Atan((FlightAssist.transPose.speedForward - setSpeed) / GyroResponsiveness) / HalfPi * MaxPitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.speedRight / GyroResponsiveness) / HalfPi * MaxRoll;
        break;

      default: // Stationary Hover
        desiredPitch = Math.Atan(FlightAssist.transPose.speedForward / GyroResponsiveness) / HalfPi * MaxPitch;
        desiredRoll = Math.Atan(FlightAssist.transPose.speedRight / GyroResponsiveness) / HalfPi * MaxRoll;
        break;
    }

    // Scale gyro rate based on difference bewteen the current and desired angle
    pitchRate = (float)(FlightAssist.blockManager.gyro.GetMaximum<float>("Pitch") * (desiredPitch - FlightAssist.transPose.pitch) / 90);
    if (pitchRate > -minRPM && pitchRate < minRPM)
      pitchRate = (float)(Math.Sign(pitchRate) * minRPM);

    rollRate = (float)(FlightAssist.blockManager.gyro.GetMaximum<float>("Roll") * (desiredRoll - FlightAssist.transPose.roll) / 90);
    if (rollRate > -minRPM && rollRate < minRPM)
      rollRate = (float)(Math.Sign(rollRate) * minRPM);

    // Transform rotation to match the remote control block's orientation rather than the "build" orientation
    Matrix shipOrientation;
    FlightAssist.blockManager.remote.Orientation.GetMatrix(out shipOrientation);
    Vector3 rotationVec = new Vector3(pitchRate, 0, rollRate);
    rotationVec = Vector3.Transform(rotationVec, shipOrientation);

    FlightAssist.blockManager.SetGyros(rotationVec, FlightAssist.blockManager.gyro.GyroOverride);
  }
}

class VectorControl : Module {
  private float pitchRate, rollRate;

  public VectorControl(FlightAssist fa, string n) : base(fa, n) {}

  override public void Tick() {
    //ExecuteManeuver();
  }

  override public void ProcessCommand(string[] args) {}

  private void ExecuteManeuver() {
    pitchRate = (float)(FlightAssist.blockManager.gyro.GetMaximum<float>("Pitch") * -FlightAssist.transPose.pitch / 90);
    if (pitchRate > -minRPM && pitchRate < minRPM)
      pitchRate = (float)(Math.Sign(pitchRate) * minRPM);

    rollRate = (float)(FlightAssist.blockManager.gyro.GetMaximum<float>("Roll") * -FlightAssist.transPose.roll / 90);
    if (rollRate > -minRPM && rollRate < minRPM)
      rollRate = (float)(Math.Sign(rollRate) * minRPM);

    if (FlightAssist.transPose.speed < 0.5) {
      rollRate = 0;
      pitchRate = 0;
    }

    // Transform rotation to match the remote control block's orientation rather than the "build" orientation
    Matrix shipOrientation;
    FlightAssist.blockManager.remote.Orientation.GetMatrix(out shipOrientation);
    Vector3D rotationVec = new Vector3(pitchRate, -rollRate, 0);
    rotationVec = Vector3D.Transform(rotationVec, shipOrientation);

    FlightAssist.blockManager.SetGyros(rotationVec, FlightAssist.blockManager.gyro.GyroOverride);
  }
}
