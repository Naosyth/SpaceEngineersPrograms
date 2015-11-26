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
int GyroResponsiveness = 2; // Any positive non-zero integer 
int GyroCount = 3; // Number of gyros to use for auto hover

// Blocks
IMyRemoteControl Remote;
IMyGyro Gyro; // This gyro is used to control enable/disable state, so that the state can be printed to the text panel
IMyTextPanel Screen;
List<IMyGyro> Gyros;

void Main() {
  if (Remote == null)
    Initialize();

  StopVehicle();
}

void Initialize() {
  Remote = GridTerminalSystem.GetBlockWithName(RemoteControlName) as IMyRemoteControl;
  Gyro = GridTerminalSystem.GetBlockWithName(GyroName) as IMyGyro;

  if (!String.IsNullOrEmpty(TextPanelName))
    Screen = GridTerminalSystem.GetBlockWithName(TextPanelName) as IMyTextPanel;

  var l = new List<IMyTerminalBlock>();
  GridTerminalSystem.GetBlocksOfType<IMyGyro>(l, x => x.CubeGrid == Me.CubeGrid && x != Gyro);
  Gyros = l.ConvertAll(x => (IMyGyro)x);
  Gyros.Insert(0, Gyro); // Make sure the master gyro is included after truncation
  Gyros = Gyros.GetRange(0, GyroCount);
}

void StopVehicle() {
  // Get current time and change in time
  double dt = 1000 / 60;

  // Get delta position
  Vector3 pos = Remote.GetPosition();
  Vector3 deltaPos = pos - oldPos;
  oldPos = pos;

  double speed = deltaPos.Length() / dt * 1000;

  // Get orientation vectors
  Matrix orientation = Remote.WorldMatrix;
  Vector3 normalVec = orientation.Up;
  Vector3 forwardVec = orientation.Forward;
  Vector3 rightVec = orientation.Right;

  // Determine speed forward and sideways in m/s
  double speedForward = Vector3.Dot(deltaPos, forwardVec) / dt * 1000;
  double speedRight = Vector3.Dot(deltaPos, rightVec) / dt * 1000;
  double speedUp = Vector3.Dot(deltaPos, normalVec) / dt * 1000;

  // Get gravity vector
  Vector3 gravityVec = -Vector3.Normalize(Remote.GetNaturalGravity());
  double gravity = Remote.GetNaturalGravity().Length() / 9.81;

  if (float.IsNaN(gravityVec.GetDim(0))) {
    if (Screen != null) {
      Screen.WritePublicText("No Gravity" + 
        "\n\nSpeed: " + String.Format("{0:000.0}", speed) + " m/s" +
        "\nForward: " + String.Format("{0:000.0}", speedForward) + " m/s" +
        "\nRight: " + String.Format("{0:000.0}", speedRight) + " m/s" +
        "\nUp: " + String.Format("{0:000.0}", speedUp) + " m/s");
    }
    return;
  }

  // Determine roll and pitch in degrees
  double pitch = Vector3.Dot(gravityVec, forwardVec) / (gravityVec.Length() * forwardVec.Length()) * 90;
  double roll = Vector3.Dot(gravityVec, rightVec) / (gravityVec.Length() * rightVec.Length()) * 90;

  // Scale max pitch based on speed
  double desiredPitch = Math.Atan(speedForward / GyroResponsiveness) / (Math.PI / 2) * MaxPitch;
  double desiredRoll = Math.Atan(speedRight / GyroResponsiveness) / (Math.PI / 2) * MaxRoll;

  float pitchRate = (float) (Gyro.GetMaximum<float>("Pitch") * (desiredPitch - pitch) /  MaxPitch);
  float rollRate = (float) (Gyro.GetMaximum<float>("Roll") * (desiredRoll - roll) /  MaxRoll);
  Vector3 rotationVec = new Vector3(pitchRate, 0, rollRate);

  // Transform rotation to match the remote control block's orientation rather than the "build" orientation
  Matrix shipOrientation;
  Remote.Orientation.GetMatrix(out shipOrientation);
  rotationVec = Vector3.Transform(rotationVec, shipOrientation);

  Matrix localOrientation;
  for (int i = 0; i < Gyros.Count; i++) {
    var g = Gyros[i];
    g.Orientation.GetMatrix(out localOrientation);

    var rotVec = Vector3.Transform(rotationVec, MatrixD.Transpose(localOrientation));

    Gyros[i].SetValueFloat("Pitch", (float)rotVec.GetDim(0));
    Gyros[i].SetValueFloat("Yaw", (float)-rotVec.GetDim(1));
    Gyros[i].SetValueFloat("Roll", (float)-rotVec.GetDim(2));
    Gyros[i].SetValueBool("Override", Gyro.GyroOverride);
  }

  // Print status
  if (Screen != null) {
    Screen.WritePublicText(
      "Speed: " + String.Format("{0:000.0}", speed) + " m/s" +
      "\nForward: " + String.Format("{0:000.0}", speedForward) + " m/s" +
      "\nRight: " + String.Format("{0:000.0}", speedRight) + " m/s" +
      "\nUp: " + String.Format("{0:000.0}", speedUp) + " m/s" +
      "\n\nRoll: " + String.Format("{0:0}", roll) + " degrees" +
      "\nPitch: " + String.Format("{0:0}", pitch) + " degrees" +
      "\n\nGravity: " + String.Format("{0:0.00}", gravity) + " g" +
      "\n\nAuto Braking: " + (Gyro.GyroOverride ? "Enabled" : "Disabled"));
  }
}
