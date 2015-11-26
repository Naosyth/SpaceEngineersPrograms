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
// GyroSpeedScale
//   Scalar which affects how fast the gyros rotate.
//   Large number means they will move faster, but be more likely to overshoot and wobble back and forth
//   This number depends on several factors including ship mass and gyro count
//   As mass increases, GyroSpeedScale should increase.
//   As gyro count increases, GyroSpeedScale should decrease.
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
string GyroName = "AH Gyro";
string RemoteControlName = "AH Remote";
string TextPanelName = "Screen Two";

// T-1 Variables
Vector3 oldPos = new Vector3(0.0, 0.0, 0.0);

// Control constants
double MaxPitch = 67.5;
double MaxRoll = 67.5;
int GyroResponsiveness = 4; // Larger = more gradual angle drop
int GyroCount = 3; // Number of gyros to use for auto hover

// Blocks
IMyRemoteControl remote;
IMyGyro gyro; // Main gyro, used to control auto hover on/off status
IMyTextPanel screen;
List<IMyGyro> gyros;

bool initialized = false;

// Can be set by passing argument to programming block
int desiredSpeedForward = 0;

void Main(string arguments) {
  if (!initialized)
    Initialize();

  // May add more args later
  if (arguments != "") {
    var args = arguments.Split(',');
    var setForward = int.Parse(args[0]);
    desiredSpeedForward = (setForward == 0 ? 0 : desiredSpeedForward + setForward);
  }

  StopVehicle();
}

void Initialize() {
  remote = GridTerminalSystem.GetBlockWithName(RemoteControlName) as IMyRemoteControl;
  gyro = GridTerminalSystem.GetBlockWithName(GyroName) as IMyGyro;

  if (!String.IsNullOrEmpty(TextPanelName))
    screen = GridTerminalSystem.GetBlockWithName(TextPanelName) as IMyTextPanel;

  var list = new List<IMyTerminalBlock>();
  GridTerminalSystem.GetBlocksOfType<IMyGyro>(list, x => x.CubeGrid == Me.CubeGrid && x != gyro);
  gyros = list.ConvertAll(x => (IMyGyro)x);
  gyros.Insert(0, gyro); // Make sure the master gyro is head of list
  gyros = gyros.GetRange(0, GyroCount);

  initialized = true;
}

void StopVehicle() {
  // Get current time and change in time
  double dt = 1000 / 60;

  // Get delta position
  Vector3 pos = remote.GetPosition();
  Vector3 deltaPos = pos - oldPos;
  oldPos = pos;
  double speed = deltaPos.Length() / dt * 1000;

  // Get orientation vectors
  Matrix orientation = remote.WorldMatrix;
  Vector3 upVec = orientation.Up;
  Vector3 forwardVec = orientation.Forward;
  Vector3 rightVec = orientation.Right;

  // Determine speed forward and sideways in m/s
  double speedForward = Vector3.Dot(deltaPos, forwardVec) / dt * 1000;
  double speedRight = Vector3.Dot(deltaPos, rightVec) / dt * 1000;
  double speedUp = Vector3.Dot(deltaPos, upVec) / dt * 1000;

  // Get gravity vector
  Vector3 gravityVec = -Vector3.Normalize(remote.GetNaturalGravity());
  double gravity = remote.GetNaturalGravity().Length() / 9.81;

  if (float.IsNaN(gravityVec.GetDim(0))) {
    if (screen != null) {
      screen.WritePublicText(
        "> Auto Hover [No Gravity]" +
        "\n----- Velocity ----------------------------------------" +
        "\nTotal: " + String.Format("{0:000}", speed) + " m/s" +
        "\n  F/B: " + String.Format("{0:000}", speedForward) +
        "\n  R/L: " + String.Format("{0:000}", speedRight) +
        "\n  U/D: " + String.Format("{0:000}", speedUp));
    }

    // Automatically disable gyro override if no gravity is present
    if (gyro.GyroOverride) {
      for (int i = 0; i < gyros.Count; i++)
        gyros[i].SetValueBool("Override", false);
    }

    return;
  }

  // Determine roll and pitch in degrees
  double pitch = Vector3.Dot(gravityVec, forwardVec) / (gravityVec.Length() * forwardVec.Length()) * 90;
  double roll = Vector3.Dot(gravityVec, rightVec) / (gravityVec.Length() * rightVec.Length()) * 90;

  // Scale desired pitch based on speed
  double desiredPitch = Math.Atan((speedForward - desiredSpeedForward) / GyroResponsiveness) / (Math.PI / 2) * MaxPitch;
  double desiredRoll = Math.Atan(speedRight / GyroResponsiveness) / (Math.PI / 2) * MaxRoll;

  // Scale gyro rate based on difference bewteen the current and desired angle
  float pitchRate = (float)(gyro.GetMaximum<float>("Pitch") * (desiredPitch - pitch) /  MaxPitch);
  float rollRate = (float)(gyro.GetMaximum<float>("Roll") * (desiredRoll - roll) /  MaxRoll);

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

  // Print status
  if (screen != null) {
    screen.WritePublicText(
      "> Auto Hover [" + (gyro.GyroOverride ? "Active" : "Disabled") + "]" +
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


