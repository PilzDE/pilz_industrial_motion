# Testdatenprovider
- Sind kartesische Positionen zu speichern, so ist die kartesische Position und
 der dazugehörige Seed zu speichern.
- Positionsangaben mithilfe von Achswerten sind gegenüber den kartesischen
Positionsangaben zu bevorzugen.
- Werden weitere Kennwerte benötigt so sind diese, insofern möglich,
über Hilfsfunktionen zu berechnen. In anderen Worten, es sollen so
wenig wie möglich Extrawerte abgespeichert werden.
- Die Orientierung kann je nach Anwendungfall in Euler- oder
Quaternion-Darstellung gespeichert werden. Ist beides möglich so ist die Euler-Darstellung zu bevorzugen, da sie leichter zu interpretieren ist.
- In Zukunft sollen neue Testdaten (Positionen, etc.) so gewählt werden, dass
sie auf allen bzw. möglichst vielen Testebenen genutzt werden können. Die neu
abgelegten Testdaten sollen dem Team kommuniziert werden (z.B. im Daily) um
nochmal im Team über die Güte der neuen Daten diskutieren zu können.

## Diagrams

### Robot configurations
![RobotConfigurations](diagrams/diag_class_robot_configurations.png)

### Command types
![Commands](diagrams/diag_class_commands.png)

### Circ auxiliary types
![AuxiliaryTypes](diagrams/diag_class_circ_auxiliary.png)
