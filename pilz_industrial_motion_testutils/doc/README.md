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

# Refactoring TestDataProvider
## Builder free variant

### Advantage
- No Builder or Director classes
- Transformation to request directly via Ptp-Class function call

### Disadvantages
- Ptp class contains information only needed for request building (like joint_prefix)
- "pure" data class Ptp has to change when request building changes or new conversion function are needed

![BuilderFreeVariant](diagrams/diag_seq_usage_testloader_option_builder_free.png)


## Builder variant

### Advantage
- Ptp pure data class: Data storage and possible transformations separated
- Transformation related information like "joint_prefix" stored in Builder

### Disadvantages
- Builder needed

![BuilderVariant](diagrams/diag_seq_usage_testloader_option_builder.png)

## Combination of both approaches
The builder free variant clearly breaks SRP as noted in the disadvantages above.
This would be solved with the builder approach (still would be to solve how different joint_prefixes are
handled if different robots are used).

One problem is that the type of configuration is inside the template of the command.
So if a PtpJoint (aka Ptp<JointConfiguration, JointConfiguration>) is passed to the builder the builder would
have to deduce the type of the Configuration.

By using a builder inside the PtpJoint Class the Seperation would be kept to some degree since the class itself would only
now that it can be transformed and to what but the "how" would be hidden in the builder.




## Class diagrams

### Robot configurations
![RobotConfigurations](diagrams/diag_class_robot_configurations.png)

### Robot configurations
![Commands](diagrams/diag_class_commands.png)

### Builder
![Builder](diagrams/diag_class_builder.png)
