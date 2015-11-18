// Ship status script
// Displays various information about the ship on a text panel
//
// Author: Naosyth
// naosyth@gmail.com

// Block Names
string CockpitName = "Cockpit";
string ConnectorName = "Connector";
string TextPanelName = "Screen One";
string AirVentName = "Air Vent";
string AntennaName = "Planetary Scout";
string BeaconName = "Beacon";
string ProjectorName = "Projector";

// Blocks
IMyCockpit Cockpit;
IMyShipConnector Connector;
IMyTextPanel Screen;
IMyAirVent Vent;
IMyRadioAntenna Antenna;
IMyBeacon Beacon;
IMyProjector Projector;

int bullets = 0;
int missiles = 0;

void Main() {
  if (Cockpit == null)
    Initialize();

  DisplayComponentStatus();
  DisplayOxygenStatus();
  DisplayComStatus();
  DisplayAmmoCount();
}

void Initialize() {
  Cockpit = GridTerminalSystem.GetBlockWithName(CockpitName) as IMyCockpit;
  Connector = GridTerminalSystem.GetBlockWithName(ConnectorName) as IMyShipConnector;
  Screen = GridTerminalSystem.GetBlockWithName(TextPanelName) as IMyTextPanel;
  Vent = GridTerminalSystem.GetBlockWithName(AirVentName) as IMyAirVent;
  Antenna = GridTerminalSystem.GetBlockWithName(AntennaName) as IMyRadioAntenna;
  Beacon = GridTerminalSystem.GetBlockWithName(BeaconName) as IMyBeacon;
  Projector = GridTerminalSystem.GetBlockWithName(ProjectorName) as IMyProjector;
}

void DisplayComponentStatus() {
  Screen.WritePublicText(
    "Dampeners: " + (Cockpit.DampenersOverride ? "Enabled" : "Disabled") +
    "\n\nConnector: " + (Connector.IsLocked ? "Locked" : "Unlocked") + 
    "\n\nProjector: " + (Projector.Enabled ? "Enabled" : "Disabled"));
}

void DisplayOxygenStatus() {
  var blocks = new List<IMyTerminalBlock>();
  GridTerminalSystem.GetBlocksOfType<IMyOxygenTank>(blocks);
  List<IMyOxygenTank> tanks = new List<IMyOxygenTank>();
  tanks = blocks.ConvertAll(x => (IMyOxygenTank)x);

  Screen.WritePublicText(
    "\n\nPressurized: " + (Vent.IsDepressurizing ? "No" : "Yes"),
    true);

  for (int i = 0; i < tanks.Count; i++) {
    Screen.WritePublicText(
      "\nO2 Tank " + (i+1) + ": " + tanks[i].GetOxygenLevel() * 100 + "%",
      true);
  }
}

void DisplayComStatus() {
  Screen.WritePublicText(
    "\n\nAntenna: " + (Antenna.Enabled ? Antenna.Radius + " m" : "Disabled") + 
    "\nBeacon: " + (Beacon.Enabled ? Beacon.Radius + " m" : "Disabled"),
    true);
}

void DisplayAmmoCount() {
  missiles = 0;
  bullets = 0;

  List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
  GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(blocks, x => x is IMyInventoryOwner);
  for (int i = 0; i < blocks.Count; i++) {
    var items = blocks[i].GetInventory(0).GetItems();
    for (int j = 0; j < items.Count; j++) {
      var item = items[j];
      if (item.Content.SubtypeName == "NATO_25x184mm")
        bullets += (int)item.Amount * 25;
      if (item.Content.SubtypeName == "Missile200mm")
        missiles += (int)item.Amount;
    }
  }

  Screen.WritePublicText(
    "\n\nMissiles: " + missiles +
    "\nBullets: " + bullets,
    true);
}
