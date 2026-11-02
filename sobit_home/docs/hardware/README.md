# Hardware wiring docs

The wiring of SOBIT HOME as code. Edit the YAML, run `tools/render.sh`, commit the YAML and the SVGs together.

```
library.yml      device templates: type, ratings, pin labels (one entry per part model)
harness/*.yml    one WireViz file per subsystem, numbered in power order (00 trunk … 80 video)
generated/       rendered SVGs, one per harness plus all.svg
tools/render.sh  renders everything; needs graphviz and wireviz 0.4.1
```

## Rules

- Each cable is defined in exactly one harness file. Shared nodes (batteries, towers, E-stop, converters, hubs) may appear in several files; the trunk file owns the cables between them, subsystem files start at a tower or hub pin.
- Part facts (voltage, current, pin labels, connector) live in `library.yml` only. A harness file names instances as `TEMPLATE.DESIGNATOR` on first use, then by designator.
- Unknown values stay empty. Do not guess a rating, a gauge or a length; a missing number is a to-do, a wrong one is a fault.
- Dynamixel IDs, baud rates and serial ports are not repeated here; they live in `sobit_home_description` (`controllers.urdf.xacro`, `robots/*.xacro`).

## Designators

`G` battery, `F` fuse adapter, `T` DC-DC converter, `S` emergency stop, `X` power tower, distribution board or terminal (`XG` star ground), `P` power or USB hub, `A` interface adapter (U2D2, USB-CAN, gateways), `K` computer, `N` network device, `W` cable. Actuators and sensors use their joint or frame name (`arm_left_elbow_a`, `wheel_drive_f_l_esc`, `lidar_front`). Cable numbers follow the file: `W0nn` trunk, `W1nn` compute, `W2nn` arms, head and shoulders, `W3nn` base, and so on.

Wire colours are IEC 60757 codes used as a drawing convention per rail (BK GND, BN 36 V, RD 24 V, OG 18 V, PK 12 V, YE 5 V). They are not the measured insulation colours.

## Power and ground

- Four battery domains: G1 (36 V: X0 → T1 → X1, X0 → T1b → P1), G2 (X2), G3 (X3), G4 (compute, X4 → T4 → X5). Negative rails never pass through the E-stop; each E-stop pole cuts one converter output.
- Star ground `XG`: one GND wire per battery domain, from that domain's root tower (X0, X2, X3, X4; cables W031–W034). Towers and hubs behind a non-isolated converter (X1, P1, X5, X6) get no wire: they reach XG through the converter's common negative, and a second wire would close a loop. If a converter is ever replaced by an isolated model, its output then needs its own wire to XG.
- Never daisy-chain ground between towers; every bond goes to XG.

## Data links

- RS-485 Dynamixel links that only carry data (U2D2 → power hub board → first servo, links between power groups, steering links) are two wires, D+ and D−, on purpose. The transceivers get their common reference through XG. Do not add a GND wire to these links: it would run in parallel with XG and form a loop. Four-wire links inside one power group carry GND and VDD because the power needs its return.
- TTL Dynamixel links (hands) keep their GND wire; TTL is single-ended.
- CAN links to the C620 ESCs carry CAN_H and CAN_L only. The SH-C31G USB-CAN adapter is isolated, so its CAN-side GND is not the NUC ground.
- Dynamixel chains run in ID order, lowest ID first.

## Rendering

```bash
sudo apt install graphviz
pip install wireviz==0.4.1        # or: uv venv wv && uv pip install --python wv/bin/python wireviz==0.4.1
sobit_home/docs/hardware/tools/render.sh   # WIREVIZ=path/to/wireviz overrides the binary
```
