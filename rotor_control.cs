// Block Names
string TextPanelName = "Text Panel";
string RotorName = "Rotor 1";

// Blocks
IMyTextPanel Screen;
IMyMotor Rotor;

void Main() {
  if (Screen == null)
    Initialize();

  AdjustRotors();
}

void Initialize() {
  Rotor = GridTerminalSystem.GetBlockWithName(RotorName) as IMyMotor;

  if (!String.IsNullOrEmpty(TextPanelName))
    Screen = GridTerminalSystem.GetBlockWithName(TextPanelName) as IMyTextPanel;
}

class RotaryThruster {
  string axis;
  bool reversed;
}
