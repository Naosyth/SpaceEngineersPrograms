// Helicopter Braking Script
// Brings a planetary flying vehicle that only has upward thrust to a stop
//
// Author: Naosyth
// naosyth@gmail.com

// T-1 Variables
Vector3 oldPos = new Vector3(0.0, 0.0, 0.0);
long oldTime = System.DateTime.Now.Ticks;

// Control constants
double setPointForward = 0.0;
double setPointRight = 0.0;
double maxPitch = 45;
double maxRoll = 45;

// Blocks
IMyProgrammableBlock computer;
IMyRemoteControl remote;
IMyTimerBlock timer;
IMyGyro gyro;

void Main() {
  if (computer == null)
    Initialize();

  StopVehicle();
}

void Initialize() {
  computer = GridTerminalSystem.GetBlockWithName("PID") as IMyProgrammableBlock;
  gyro = GridTerminalSystem.GetBlockWithName("Gyro") as IMyGyro;
  remote = GridTerminalSystem.GetBlockWithName("Remote") as IMyRemoteControl;
  timer = GridTerminalSystem.GetBlockWithName("Timer") as IMyTimerBlock;
}

void StopVehicle() {
  // Get current time and change in time
  long now = System.DateTime.Now.Ticks / System.TimeSpan.TicksPerMillisecond;
  double dt = now - oldTime;

  // Get current position
  Vector3 computerPos = computer.GetPosition();
  Vector3 remotePos = remote.GetPosition();
  Vector3 timerPos = timer.GetPosition();
  Vector3 gyroPos = gyro.GetPosition();

  // Get delta position
  Vector3 deltaPos = computerPos - oldPos;
  double speed = deltaPos.Length() / dt * 1000;

  // Get orientation vectors
  Vector3 normalVec = Vector3.Normalize(remotePos - computerPos);
  Vector3 forwardVec = Vector3.Normalize(gyroPos - computerPos);
  Vector3 rightVec = Vector3.Normalize(timerPos - computerPos);

  // Get gravity vector
  Vector3 gravityVec = Vector3.Multiply(Vector3.Normalize(remote.GetNaturalGravity()), -1);

  // Determine speed forward and sideways in m/s
  double speedForward = Vector3.Dot(deltaPos, forwardVec) / dt * 1000;
  double speedRight = Vector3.Dot(deltaPos, rightVec) / dt * 1000;

  // Determine roll and pitch in degrees
  double pitch = Vector3.Dot(gravityVec, forwardVec) / (gravityVec.Length() * forwardVec.Length()) * 90;
  double roll = Vector3.Dot(gravityVec, rightVec) / (gravityVec.Length() * rightVec.Length()) * 90;

  double scaledMaxPitch = Math.Atan(speedForward / 4) / (Math.PI / 2) * maxPitch;
  double scaledMaxRoll = Math.Atan(speedRight / 4) / (Math.PI / 2) * maxRoll;

  float pitchRate = (float)((Math.Abs(pitch - scaledMaxPitch) / maxPitch) * 4);
  float rollRate = (float)((Math.Abs(roll - scaledMaxRoll) / maxRoll) * 4);

  if (Math.Abs(speedForward) < 0.01)
    gyro.SetValueFloat("Pitch", 0.0f);
  else if (pitch < scaledMaxPitch)
    gyro.SetValueFloat("Pitch", pitchRate);
  else
    gyro.SetValueFloat("Pitch", -1 * pitchRate);

  if (Math.Abs(speedRight) < 0.01)
    gyro.SetValueFloat("Roll", 0.0f);
  else if (roll < scaledMaxRoll)
    gyro.SetValueFloat("Roll", -1 * rollRate);
  else
    gyro.SetValueFloat("Roll", rollRate);


  // Get screen for debugging purposes
  var screens = new List<IMyTerminalBlock>();
  GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(screens);
  if (screens.Count > 0) {
    IMyTextPanel screen = screens[0] as IMyTextPanel;

    // Print debug info
    screen.WritePublicText(
      "Speed: " + speed +
      "\nForward: " + speedForward +
      "\nRight: " + speedRight +
      "\n\nroll: " + roll +
      "\npitch: " + pitch +
      "\nmax roll: " + scaledMaxRoll +
      "\nmax pitch: " + scaledMaxPitch +
      "\n\ndt: " + dt);
  }

  // Persist variables
  oldTime = now;
  oldPos = computerPos;
}
