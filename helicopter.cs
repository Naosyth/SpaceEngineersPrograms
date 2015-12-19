// Helicopter Braking Script
// Brings a planetary flying vehicle that only has upward thrust to a stop
//
// Author: Naosyth
// naosyth@gmail.com

// ==================== //
// === Tuning Guide === //
// ==================== //
// Four main variables control how the script behaves
// 
// MaxPitch and MaxRoll
//   Sets the maximum angle in degrees that the ship will rotate to bring itself to a stop
//   A high angle means you will stop faster, but also have more vertical movement.
//
// GyroResponsiveness
//   This controls the curve of the angle the ship takes while coming to a stop.
//   The curve looks something like this:
//   http://www.wolframalpha.com/input/?i=arctan%28x%2F4%29+%2F+%283.14159+%2F+2%29+for+x+from+-104+to+104
//   Where 4, in the above equasion, is GyroResponsiveness
//   Increasing GyroResponsiveness makes the angle start approaching zero earlier
//   This can help to reduce wobble, but also means you will take a bit longer to slow down.
//

// Block Names
static string GyroName = "AH Gyro";
static string RemoteControlName = "AH Remote";
static string TextPanelName = "AH Screen";

// Program options
static int ScreenRefreshRate = 5; // How many times per second to redraw the screen
static bool AlwaysEnabledInGravity = false;

// Control constants
static double MaxPitch = 67.5;
static double MaxRoll = 67.5;
static int GyroResponsiveness = 8; // Larger = more gradual angle drop
static int GyroCount = 3; // Number of gyros to use for auto hover
static double minRPM = 0.015; // Min RPM setting for gyros. Values that are too low behave weird. You shouldn't need to adjust this.

const double HalfPi = Math.PI / 2;

AutoHoverController controller;

void Main(string arguments) {
  if (controller == null)
    controller = new AutoHoverController(GridTerminalSystem, Me);

  if (arguments != "") {
    var args = arguments.Split(' ');
    var command = args[0].ToLower();

    if (command == "mode") {
      var mode = args[1];
      controller.mode = mode;

      if (args.Length >= 3)
        controller.setSpeed = Int32.Parse(args[2]);
    }
  }

  controller.Tick();
}

class AutoHoverController {
  private IMyProgrammableBlock Me;
  private IMyGridTerminalSystem  GridTerminalSystem;

  private IMyRemoteControl remote;
  private IMyTextPanel screen;
  private List<IMyGyro> gyros;
  private IMyGyro gyro;

  private Vector3 pos;
  private Vector3 oldPos;

  private double speed;
  private double speedForward;
  private double speedRight;
  private double speedUp;
  private double controlSpeedForward;
  private double controlSpeedRight;

  private Vector3 forwardVec;
  private Vector3 rightVec;
  private Vector3 upVec;

  private bool inGravity;
  private double gravity;

  private double pitch;
  private double desiredPitch;
  private float pitchRate;

  private double roll;
  private double desiredRoll;
  private float rollRate;

  public string mode;
  public int setSpeed;

  private int tickCount;

  public AutoHoverController(IMyGridTerminalSystem gts, IMyProgrammableBlock pb) {
    Me = pb;
    GridTerminalSystem = gts;

    remote = GridTerminalSystem.GetBlockWithName(RemoteControlName) as IMyRemoteControl;
    gyro = GridTerminalSystem.GetBlockWithName(GyroName) as IMyGyro;

    if (!String.IsNullOrEmpty(TextPanelName))
      screen = GridTerminalSystem.GetBlockWithName(TextPanelName) as IMyTextPanel;

    var list = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyGyro>(list, x => x.CubeGrid == Me.CubeGrid && x != gyro);
    gyros = list.ConvertAll(x => (IMyGyro)x);
    gyros.Insert(0, gyro);
    gyros = gyros.GetRange(0, GyroCount);

    mode = "Hover";
    setSpeed = 0;
  }

  public void Tick() {
    CalculateOrientation();
    CalculateSpeed();

    if (!inGravity && gyro.GyroOverride)
      ToggleGyros(false);
    else if (AlwaysEnabledInGravity && inGravity && !gyro.GyroOverride)
      ToggleGyros(true);
    else
      AdjustOrientation();

    if (tickCount % (60 / ScreenRefreshRate) == 0)
      PrintStatus();

    tickCount += 1;
  }

  private void CalculateSpeed() {
    // Get current time and change in time
    double dt = 1000 / 60;

    // Get delta position
    pos = remote.GetPosition();
    Vector3 deltaPos = pos - oldPos;
    oldPos = pos;
    speed = deltaPos.Length() / dt * 1000;

    // Get orientation vectors
    Matrix orientation = remote.WorldMatrix;
    forwardVec = orientation.Forward;
    rightVec = orientation.Right;
    upVec = orientation.Up;

    // Determine speed forward and sideways in m/s
    if (inGravity) {
      Vector3 gravityVec = -Vector3.Normalize(remote.GetNaturalGravity());
      speedForward = Vector3.Dot(deltaPos, Vector3.Cross(gravityVec, rightVec)) / dt * 1000;
      speedRight = Vector3.Dot(deltaPos, -Vector3.Cross(gravityVec, forwardVec)) / dt * 1000;
      speedUp = Vector3.Dot(deltaPos, gravityVec) / dt * 1000;
    } else {
      speedForward = Vector3.Dot(deltaPos, forwardVec) / dt * 1000;
      speedRight = Vector3.Dot(deltaPos, rightVec) / dt * 1000;
      speedUp = Vector3.Dot(deltaPos, upVec) / dt * 1000;
    }
  }

  private void CalculateOrientation() {
    // Get gravity vector
    Vector3 gravityVec = -Vector3.Normalize(remote.GetNaturalGravity());
    gravity = remote.GetNaturalGravity().Length() / 9.81;
    inGravity = !float.IsNaN(gravityVec.GetDim(0));

    // Determine roll and pitch in degrees
    if (inGravity) {
      pitch = Vector3.Dot(gravityVec, forwardVec) / (gravityVec.Length() * forwardVec.Length()) * 90;
      roll = Vector3.Dot(gravityVec, rightVec) / (gravityVec.Length() * rightVec.Length()) * 90;
    }
  }

  private void AdjustOrientation() {
    switch (mode.ToLower()) {
      case "glide":
        desiredPitch = 0;
        desiredRoll = Math.Atan(speedRight / GyroResponsiveness) / HalfPi * MaxRoll;
        break;

      case "freeglide":
        desiredPitch = 0;
        desiredRoll = 0;
        break;

      case "cruise":
        desiredPitch = Math.Atan((speedForward - setSpeed) / GyroResponsiveness) / HalfPi * MaxPitch;
        desiredRoll = Math.Atan(speedRight / GyroResponsiveness) / HalfPi * MaxRoll;
        break;

      default: // Stationary Hover
        desiredPitch = Math.Atan(speedForward / GyroResponsiveness) / HalfPi * MaxPitch;
        desiredRoll = Math.Atan(speedRight / GyroResponsiveness) / HalfPi * MaxRoll;
        break;
    }

    // Scale gyro rate based on difference bewteen the current and desired angle
    pitchRate = (float)(gyro.GetMaximum<float>("Pitch") * (desiredPitch - pitch) / 90);
    if (pitchRate > -minRPM && pitchRate < minRPM)
      pitchRate = (float)(Math.Sign(pitchRate) * minRPM);

    rollRate = (float)(gyro.GetMaximum<float>("Roll") * (desiredRoll - roll) / 90);
    if (rollRate > -minRPM && rollRate < minRPM)
      rollRate = (float)(Math.Sign(rollRate) * minRPM);

    // Transform rotation to match the remote control block's orientation rather than the "build" orientation
    Matrix shipOrientation;
    remote.Orientation.GetMatrix(out shipOrientation);
    Vector3 rotationVec = new Vector3(pitchRate, 0, rollRate);
    rotationVec = Vector3.Transform(rotationVec, shipOrientation);

    for (int i = 0; i < gyros.Count; i++) {
      var g = gyros[i];

      // Adjust rotation for the gyro's local orientation
      Matrix localOrientation;
      g.Orientation.GetMatrix(out localOrientation);
      var localRot = Vector3.Transform(rotationVec, MatrixD.Transpose(localOrientation));

      g.SetValueFloat("Pitch", (float)localRot.GetDim(0));
      g.SetValueFloat("Yaw", (float)-localRot.GetDim(1));
      g.SetValueFloat("Roll", (float)-localRot.GetDim(2));
      g.SetValueBool("Override", gyro.GyroOverride);
    }
  }

  private void PrintStatus() {
    if (screen != null) {
      if (!inGravity) {
        screen.WritePublicText(
          "> Auto Hover [No Gravity]" +
          "\n----- Velocity ----------------------------------------" +
          "\nTotal: " + String.Format("{0:000}", speed) + " m/s" +
          "\n  F/B: " + String.Format("{0:000}", speedForward) +
          "\n  R/L: " + String.Format("{0:000}", speedRight) +
          "\n  U/D: " + String.Format("{0:000}", speedUp));

        return;
      }

      screen.WritePublicText(
        "> Auto Hover [" + (gyro.GyroOverride ? "Active" : "Disabled") + "] [" + mode + "]" +
        "\n----- Velocity ----------------------------------------" +
        "\nTotal: " + String.Format("{0:000}", speed) + " m/s" +
        "\n  F/B: " + String.Format("{0:000}", speedForward) +
        "\n  R/L: " + String.Format("{0:000}", speedRight) +
        "\n  U/D: " + String.Format("{0:000}", speedUp) +
        "\n\n----- Orientation ----------------------------------------" +
        "\nPitch: " + String.Format("{0:00}", pitch) + "°" +
        " | Roll: " + String.Format("{0:00}", roll) + "°" +
        "\n\n----- Status ----------------------------------------" +
        "\nGravity: " + String.Format("{0:0.00}", gravity) + " g");
    }
  }

  private void ToggleGyros(bool state) {
    for (int i = 0; i < gyros.Count; i++)
      gyros[i].SetValueBool("Override", state);
  }
}
