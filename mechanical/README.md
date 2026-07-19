# Mechanical design files

`solidworks/` contains the native SolidWorks parts and assemblies recovered from the original archive/local project directory.

![CAD assembly](../docs/assets/cad-assembly.png)

## Suggested entry points

- `solidworks/装配体（水平推进器简化）.SLDASM` — simplified horizontal-thruster assembly;
- `solidworks/外径130x300密封舱总装2.SLDASM` — pressure-housing assembly;
- `solidworks/分隔板+连接件.SLDASM` — internal divider assembly;
- `solidworks/头部圆壳.SLDPRT` — nose structure;
- `solidworks/尾部垂推连接件1.SLDPRT` and `尾部垂推连接件2.SLDPRT` — rear vertical-thruster mounts.

File names preserve the original Chinese names so SolidWorks assembly references are less likely to break.

## Reported prototype data

| Property | Reported value |
| --- | ---: |
| External assembly mass | approximately 3.451 kg |
| Displaced volume | approximately 2.387 × 10⁶ mm³ |
| Estimated buoyancy | approximately 50.8 N |
| Centre-of-mass Z | 223.254 mm in the report's CAD frame |
| Centre-of-buoyancy Z | 230.746 mm in the report's CAD frame |
| Vertical separation | approximately 7.49 mm, buoyancy centre above mass centre |

These are report snapshots, not independently recalculated values. The directory includes names referring to both 250 mm and 300 mm housings; confirm the active assembly, units, configurations, missing references, and final manufactured dimensions before producing parts.

## Recommended workflow

1. Open the top assembly in the same or a newer SolidWorks version.
2. Resolve all references without renaming source files.
3. Confirm document units and material assignments.
4. Recalculate mass properties with the exact battery, electronics, fasteners, seals, and wiring.
5. Export neutral formats and drawings for fabrication in a reviewed release rather than treating native CAD alone as manufacturing documentation.

The files do not establish a certified pressure rating. See `docs/safety.md` before leak or immersion testing.
