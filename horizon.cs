ArtificialHorizon artificialHorizon;
bool initialized = false;

const double RadToDeg = 180 / Math.PI;

void Main() {
  if (!initialized) {
    artificialHorizon = new ArtificialHorizon(GridTerminalSystem);
    initialized = true;
  }

  artificialHorizon.Tick();
}

class ArtificialHorizon {
  private string remoteControlName = "Horizon Remote";
  private IMyRemoteControl remote;

  private string textPanelBlockName = "Horizon Screen";
  private IMyTextPanel screen;

  private int screenDrawInterval = 5;

  private int height = 15;
  private int width = 27;

  private Vector3D oldPosition;
  private Vector3D position;
  private Vector3D deltaPosition;
  private double speed;
  private double worldSpeedForward, worldSpeedRight, worldSpeedUp;
  private double localSpeedForward, localSpeedRight, localSpeedUp;

  private Matrix shipOrientation;
  private Matrix worldOrientation;
  private Vector3D rotationVec;
  private Vector3D forwardVec, rightVec, upVec;
  private double pitch, tilt;

  private Vector3D gravity;
  private bool inGravity;

  private double dt = 1000/60;

  private int ticks;

  public ArtificialHorizon(IMyGridTerminalSystem gts) {
    remote = gts.GetBlockWithName(remoteControlName) as IMyRemoteControl;
    remote.Orientation.GetMatrix(out shipOrientation);

    screen = gts.GetBlockWithName(textPanelBlockName) as IMyTextPanel;
  }

  public void Tick() {
    CalcVelocity();
    CalcOrientation();
    CalcSpeedComponents();

    ticks += 1;
    if (ticks % screenDrawInterval == 0)
      PrintHorizon();
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
    inGravity = !double.IsNaN(gravity.GetDim(0));
    
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
      localSpeedForward = 0;
      localSpeedRight = 0;
      localSpeedUp = 0;
      return;
    }

    if (inGravity) {
      worldSpeedForward = Vector3D.Dot(deltaPosition, Vector3D.Cross(gravity, rightVec)) * speed;
      worldSpeedRight = Vector3D.Dot(deltaPosition, -Vector3D.Cross(gravity, forwardVec)) * speed;
      worldSpeedUp = Vector3D.Dot(deltaPosition, gravity) * speed;
    } else {
      worldSpeedForward = 0;
      worldSpeedRight = 0;
      worldSpeedUp = 0;
    }

    localSpeedForward = Vector3D.Dot(deltaPosition, forwardVec) * speed;
    localSpeedRight = Vector3D.Dot(deltaPosition, rightVec) * speed;
    localSpeedUp = Vector3D.Dot(deltaPosition, upVec) * speed;
  }

  private void PrintHorizon() {
    int yCenter = height/2;
    int xCenter = width/2;

    double pitchPoint, tiltPoint;

    if (inGravity) {
      pitchPoint = height - Math.Floor(pitch / 180 * height) - 1;
      tiltPoint = Math.Floor(Vector3D.Dot(rightVec, deltaPosition) * xCenter) + xCenter;
      if (localSpeedForward < 0)
        tiltPoint = width - tiltPoint;
    } else {
      pitchPoint = -Math.Floor((localSpeedUp / speed) * yCenter) + yCenter;
      tiltPoint = Math.Floor((localSpeedRight / speed) * xCenter) + xCenter;
    }

    double roll, horizon, horizonPoint;
    roll = Math.Asin(Vector3.Dot(gravity, rightVec)) * RadToDeg;
    horizonPoint = pitchPoint;
    horizon = pitchPoint;

    string output = "";
    for (var y = 0; y < height; y++) {
      output += "       |";
      for (var x = 0; x < width; x++) {
        var upsideDown = Vector3D.Dot(upVec, gravity) < 0;

        if (inGravity) {
          if (EqualWithMargin(roll, 0, 0.01)) {
            horizonPoint = horizon;
          } else {
            horizonPoint = Math.Floor((height * 2 * roll / 90)) * (x - xCenter) / (width/2) + horizon;
            if (upsideDown)
              horizonPoint = height - horizonPoint;
          }
        }

        if (inGravity && (x == tiltPoint && y == 0))
          output += ".!";
        else if (!inGravity && x == tiltPoint && y == pitchPoint)
          output += localSpeedForward < 0 ? "~" : "+";
        else if (x == xCenter && y == yCenter)
          output += "  ";
        else if (x == xCenter-1 && y == yCenter)
          output += "<";
        else if (x == xCenter+1 && y == yCenter)
          output += ">";
        else if (inGravity && !upsideDown && y > horizonPoint)
          output += "=";
        else if (inGravity && upsideDown && y < horizonPoint)
          output += "=";
        else
          output += ". ";
      }
      output += "|\n";
    }

    output += "       F/B: " + String.Format("{0:000}", localSpeedForward) +
              "  R/L: " + String.Format("{0:000}", localSpeedRight) +
              "  U/D: " + String.Format("{0:000}", localSpeedUp) +
              "\n       Pitch: " + String.Format("{0:00}", pitch) + "°" +
              " | Tilt: " + String.Format("{0:00}", tilt) + "°" + 
              "   Total: " + String.Format("{0:000}", speed) + " m/s";

    screen.WritePublicText(output);
  }
}

static bool EqualWithMargin(double value, double target, double margin) {
  return value > target - margin && value < target + margin;
}
