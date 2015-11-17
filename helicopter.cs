// Helicopter Braking Script
// Brings a planetary flying vehicle that only has upward thrust to a stop
//
// Author: Naosyth
// naosyth@gmail.com

// Block Names
string ComputerName = "FC Computer";
string GyroName = "FC Gyro"; // Must be positioned forward of computer
string RemoteControlName = "FC Remote"; // Above computer
string TimerName = "FC Timer"; // Right of computer
string TextPanelName = "Screen Two";

// T-1 Variables
Vector3 oldPos = new Vector3(0.0, 0.0, 0.0);
long oldTime = System.DateTime.Now.Ticks;

// Control constants
double MaxPitch = 45;
double MaxRoll = 45;

// Blocks
IMyProgrammableBlock Computer;
IMyRemoteControl Remote;
IMyTimerBlock Timer;
IMyGyro Gyro;
IMyTextPanel Screen;

void Main() {
  if (Computer == null)
    Initialize();

  StopVehicle();
}

void Initialize() {
  Computer = GridTerminalSystem.GetBlockWithName(ComputerName) as IMyProgrammableBlock;
  Remote = GridTerminalSystem.GetBlockWithName(RemoteControlName) as IMyRemoteControl;
  Timer = GridTerminalSystem.GetBlockWithName(TimerName) as IMyTimerBlock;
  Gyro = GridTerminalSystem.GetBlockWithName(GyroName) as IMyGyro;
  Screen = GridTerminalSystem.GetBlockWithName(TextPanelName) as IMyTextPanel;
}

void StopVehicle() {
  // Get current time and change in time
  long now = System.DateTime.Now.Ticks / System.TimeSpan.TicksPerMillisecond;
  double dt = now - oldTime;

  // Get current position
  Vector3 computerPos = Computer.GetPosition();
  Vector3 remotePos = Remote.GetPosition();
  Vector3 timerPos = Timer.GetPosition();
  Vector3 gyroPos = Gyro.GetPosition();

  // Get delta position
  Vector3 deltaPos = computerPos - oldPos;
  double speed = deltaPos.Length() / dt * 1000;

  // Get orientation vectors
  Vector3 normalVec = Vector3.Normalize(remotePos - computerPos);
  Vector3 forwardVec = Vector3.Normalize(gyroPos - computerPos);
  Vector3 rightVec = Vector3.Normalize(timerPos - computerPos);

  // Get gravity vector
  Vector3 gravityVec = Vector3.Multiply(Vector3.Normalize(Remote.GetNaturalGravity()), -1);

  // Determine speed forward and sideways in m/s
  double speedForward = Vector3.Dot(deltaPos, forwardVec) / dt * 1000;
  double speedRight = Vector3.Dot(deltaPos, rightVec) / dt * 1000;

  // Determine roll and pitch in degrees
  double pitch = Vector3.Dot(gravityVec, forwardVec) / (gravityVec.Length() * forwardVec.Length()) * 90;
  double roll = Vector3.Dot(gravityVec, rightVec) / (gravityVec.Length() * rightVec.Length()) * 90;

  double scaledMaxPitch = Math.Atan(speedForward / 4) / (Math.PI / 2) * MaxPitch;
  double scaledMaxRoll = Math.Atan(speedRight / 4) / (Math.PI / 2) * MaxRoll;

  float pitchRate = (float)((Math.Abs(pitch - scaledMaxPitch) / MaxPitch) * 4);
  float rollRate = (float)((Math.Abs(roll - scaledMaxRoll) / MaxRoll) * 4);

  if (Math.Abs(speedForward) < 0.01)
    Gyro.SetValueFloat("Pitch", 0.0f);
  else
    Gyro.SetValueFloat("Pitch", pitchRate * Math.Sign(scaledMaxPitch - pitch));

  if (Math.Abs(speedRight) < 0.01)
    Gyro.SetValueFloat("Roll", 0.0f);
  else
    Gyro.SetValueFloat("Roll", rollRate * Math.Sign(roll - scaledMaxRoll));

  // Get screen for debugging purposes
  if (Screen != null) {
    // Print debug info
    Screen.WritePublicText(
      "Speed: " + String.Format("{0:000.0}", speed) + " m/s" +
      "\nForward: " + String.Format("{0:000.0}", speedForward) + " m/s" +
      "\nRight: " + String.Format("{0:000.0}", speedRight) + " m/s" +
      "\n\nRoll: " + String.Format("{0:0}", roll) + " degrees" +
      "\nPitch: " + String.Format("{0:0}", pitch) + " degrees" +
      "\n\nAuto Braking: " + (Gyro.GyroOverride ? "Enabled" : "Disabled"));
  }

  // Persist variables
  oldTime = now;
  oldPos = computerPos;
}
