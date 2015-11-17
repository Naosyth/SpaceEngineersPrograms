// Helicopter Braking Script
// Brings a planetary flying vehicle that only has upward thrust to a stop
//
// Author: Naosyth
// naosyth@gmail.com

// T-1 Variables
Vector3 oldPos = new Vector3(0.0, 0.0, 0.0);
long oldTime = System.DateTime.Now.Ticks;

// PID loop parameters
double Kp = 0.5;
double Ki = 0.0;
double Kd = 5.0;
double gyroCoefficient = 2.0;

// PID variables
double oldErrForward = 0.0;
double oldErrRight = 0.0;
double integralForward = 0.0;
double integralRight = 0.0;

// Control constants
double setPointForward = 0.0;
double setPointRight = 0.0;
double maxPitch = 30;
double maxRoll = 30;

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

  // Determine roll and pitch in degrees
  double pitch = Vector3.Dot(gravityVec, forwardVec) / (gravityVec.Length() * forwardVec.Length()) * 90;
  double roll = Vector3.Dot(gravityVec, rightVec) / (gravityVec.Length() * rightVec.Length()) * 90;

  // Determine speed forward and sideways in m/s
  double speedForward = Vector3.Dot(deltaPos, forwardVec) / dt * 1000;
  double speedRight = Vector3.Dot(deltaPos, rightVec) / dt * 1000;

  // Calculate forward PID
  double errForward = setPointForward - speedForward;
  integralForward += errForward;
  double derivativeForward = (errForward - oldErrForward);
  double outForward = Kp*errForward + Ki*integralForward + Kd*derivativeForward;

  // Calculate right PID
  double errRight = setPointRight - speedRight;
  integralRight += errRight;
  double derivativeRight = (errRight - oldErrRight);
  double outRight = Kp*errRight + Ki*integralRight + Kd*derivativeRight;

  // Use output to control gyro
  if (Math.Abs(pitch) < maxPitch || Math.Sign(pitch) == Math.Sign(outForward))
    gyro.SetValueFloat("Pitch", (float)(-1*outForward*gyroCoefficient));
  else
    gyro.SetValueFloat("Pitch", 0);

  if (Math.Abs(roll) < maxRoll || Math.Sign(roll) == Math.Sign(outRight))
    gyro.SetValueFloat("Roll", (float)(1*outRight*gyroCoefficient));
  else
    gyro.SetValueFloat("Roll", 0);

  // Get screen for debugging purposes
  var screens = new List<IMyTerminalBlock>();
  GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(screens);
  if (screens.Count > 0) {
    IMyTextPanel screen = screens[0] as IMyTextPanel;

    // Print debug info
    screen.WritePublicText(
      "PID Output Forward: " + outForward +
      "\nPid Output Right: " + outRight +
      "\n\ngravity: " + gravityVec +
      "\nspeed: " + speed +
      "\nroll: " + roll +
      "\npitch: " + pitch +
      "\n\ndt: " + dt +
      "\n\nForward: " + speedForward +
      "\nRight: " + speedRight);
  }

  // Persist variables
  oldTime = now;
  oldPos = computerPos;
  oldErrForward = errForward;
  oldErrRight = errRight;
}
