"""
SpiRob generator for Autodesk Fusion 360
v3.1: Command dialog + separate components
-------------------------------------------------
Each segment is a separate component containing its own body,
positioned along +Z in the root assembly.

Run the script, fill in the dialog, press OK.
"""

import adsk.core
import adsk.fusion
import math
import traceback

_handlers = []


def mm(x):
    """Millimetres -> Fusion internal units (cm)."""
    return x / 10.0


# ============================================================
# GEOMETRY HELPERS
# ============================================================

def offset_plane(comp, z_cm):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(
        comp.xYConstructionPlane,
        adsk.core.ValueInput.createByReal(z_cm)
    )
    return comp.constructionPlanes.add(pin)


def cut_circles(comp, body, z_cm, radius_cm,
                circ_r_cm, height_cm, phase_deg, n_tendons):
    """Cut a ring of circles from a segment body."""

    plane = offset_plane(comp, z_cm)
    sk = comp.sketches.add(plane)

    for k in range(n_tendons):
        a = math.radians(phase_deg + k * 360.0 / n_tendons)
        c = adsk.core.Point3D.create(
            radius_cm * math.cos(a),
            radius_cm * math.sin(a),
            0
        )
        sk.sketchCurves.sketchCircles.addByCenterRadius(c, circ_r_cm)

    profs = adsk.core.ObjectCollection.create()
    for prof in sk.profiles:
        profs.add(prof)

    ext = comp.features.extrudeFeatures
    inp = ext.createInput(
        profs,
        adsk.fusion.FeatureOperations.CutFeatureOperation
    )
    inp.participantBodies = [body]
    inp.setDistanceExtent(
        False,
        adsk.core.ValueInput.createByReal(height_cm)
    )
    ext.add(inp)


# -------------------- SEGMENT GENERATOR --------------------

def make_segment(root, i, z0, h, d_bot, d_top, is_first, is_last, p):

    # Component occurrence at its assembly position
    transform = adsk.core.Matrix3D.create()
    transform.translation = adsk.core.Vector3D.create(0, 0, mm(z0))

    occurrence = root.occurrences.addNewComponent(transform)
    comp = occurrence.component

    name = "seg_%02d" % i
    comp.name = name

    # Local segment geometry (mm)
    z_local = 0.0
    z_top = h

    r0 = d_bot / 2.0
    r1 = d_top / 2.0
    rb = (p["ROD_D"] + p["BORE_CLEAR"]) / 2.0

    tan_a = math.tan(math.radians(p["JOINT_BEND_DEG"] / 2.0))

    z_bot_out = z_local if is_first else z_local + (r0 - rb) * tan_a
    z_top_out = z_top if is_last else z_top - (r1 - rb) * tan_a

    poly = [
        (rb, z_local),
        (r0, z_bot_out),
        (r1, z_top_out),
        (rb, z_top),
    ]
    poly = [(mm(x), mm(z)) for x, z in poly]

    sk = comp.sketches.add(comp.xZConstructionPlane)

    pts = [
        sk.modelToSketchSpace(adsk.core.Point3D.create(x, 0, z))
        for x, z in poly
    ]

    lines = sk.sketchCurves.sketchLines
    for a, b in zip(pts, pts[1:] + pts[:1]):
        lines.addByTwoPoints(a, b)

    revs = comp.features.revolveFeatures
    rin = revs.createInput(
        sk.profiles.item(0),
        comp.zConstructionAxis,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    rin.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))

    body = revs.add(rin).bodies.item(0)
    body.name = name + "_body"

    full_h = mm(h) + 0.02

    # Tendon holes
    cut_circles(
        comp, body,
        mm(z_local),
        mm(p["TENDON_R_FRAC"] * d_bot),
        mm(p["TENDON_D"]) / 2.0,
        full_h,
        p["TENDON_PHASE_DEG"],
        p["N_TENDONS"]
    )

    # Scallops
    if p["SCALLOPS"]:
        cut_circles(
            comp, body,
            mm(z_local),
            mm(p["SCALLOP_CENTER_FRAC"] * d_bot),
            mm(p["SCALLOP_R_FRAC"] * d_bot),
            full_h,
            p["TENDON_PHASE_DEG"] + 180.0 / p["N_TENDONS"],
            p["N_TENDONS"]
        )

    # Tip anchor pockets
    if is_last and p["ANCHOR_D"] > 0:
        cut_circles(
            comp, body,
            mm(z_top - p["ANCHOR_DEPTH"]),
            mm(p["TENDON_R_FRAC"] * d_bot),
            mm(p["ANCHOR_D"]) / 2.0,
            mm(p["ANCHOR_DEPTH"]) + 0.02,
            p["TENDON_PHASE_DEG"],
            p["N_TENDONS"]
        )

    return body


# --------------------- MAIN GENERATION ---------------------

def generate_spi_rob(p):

    app = adsk.core.Application.get()
    ui = app.userInterface

    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        ui.messageBox("Open a Fusion Design (Model workspace) first.")
        return

    # ---- sanity checks ----
    if p["D1"] > p["D0"]:
        ui.messageBox("Tip diameter must not exceed base diameter.")
        return

    rb = (p["ROD_D"] + p["BORE_CLEAR"]) / 2.0
    if p["D1"] / 2.0 <= rb:
        ui.messageBox(
            "Tip radius (%.2f mm) must be larger than the spine bore "
            "radius (%.2f mm)." % (p["D1"] / 2.0, rb)
        )
        return

    # Tendon holes must clear the bore at the smallest segment
    if p["TENDON_R_FRAC"] * p["D1"] - p["TENDON_D"] / 2.0 <= rb:
        ui.messageBox(
            "Tendon holes would intersect the spine bore at the tip. "
            "Increase 'Tendon radial fraction' or reduce spine/tendon size."
        )
        return

    root = design.rootComponent

    # Geometric taper
    if abs(p["D1"] - p["D0"]) < 1e-9:
        q = 1.0
        h0 = p["L"] / p["N"]
    else:
        q = (p["D1"] / p["D0"]) ** (1.0 / p["N"])
        h0 = p["L"] * (1 - q) / (1 - q ** p["N"])

    z = 0.0
    for i in range(p["N"]):
        h = h0 * q ** i
        d_bot = p["D0"] * q ** i
        d_top = p["D0"] * q ** (i + 1)

        make_segment(
            root, i, z, h, d_bot, d_top,
            i == 0, i == p["N"] - 1, p
        )

        z += h
        adsk.doEvents()

    app.activeViewport.fit()

    ui.messageBox(
        "SpiRob successfully generated!\n\n"
        "Segments: %d\n"
        "Taper ratio: %.5f\n"
        "Initial segment height: %.2f mm\n"
        "Total length: %.1f mm\n"
        "Maximum theoretical curl: %.0f deg"
        % (p["N"], q, h0, z, (p["N"] - 1) * p["JOINT_BEND_DEG"])
    )


# --------------------- FUSION COMMAND UI ---------------------

class SpiRobExecuteHandler(adsk.core.CommandEventHandler):

    def notify(self, args):
        try:
            cmd = args.firingEvent.sender
            inputs = cmd.commandInputs

            def val(name):
                return inputs.itemById(name).value

            # Fusion returns lengths in cm and angles in radians.
            p = {
                "L": val("L") * 10.0,
                "N": int(val("N")),
                "D0": val("D0") * 10.0,
                "D1": val("D1") * 10.0,

                "JOINT_BEND_DEG": math.degrees(val("BEND")),
                "ROD_D": val("ROD_D") * 10.0,
                "BORE_CLEAR": val("BORE_CLEAR") * 10.0,

                "N_TENDONS": int(val("N_TENDONS")),
                "TENDON_D": val("TENDON_D") * 10.0,
                "TENDON_R_FRAC": val("TENDON_R_FRAC"),
                "TENDON_PHASE_DEG": math.degrees(val("PHASE")),

                "SCALLOPS": int(val("SCALLOPS")) > 0,
                "SCALLOP_CENTER_FRAC": val("SCALLOP_CENTER_FRAC"),
                "SCALLOP_R_FRAC": val("SCALLOP_R_FRAC"),

                "ANCHOR_D": val("ANCHOR_D") * 10.0,
                "ANCHOR_DEPTH": val("ANCHOR_DEPTH") * 10.0,
            }

            generate_spi_rob(p)

        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox("Generation failed:\n{}".format(traceback.format_exc()))


class SpiRobDestroyHandler(adsk.core.CommandEventHandler):

    def notify(self, args):
        # Dialog closed (OK or Cancel) -> end the script
        adsk.terminate()


class SpiRobCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):

    def notify(self, args):
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            # ---------------- GEOMETRY ----------------
            inputs.addTextBoxCommandInput("geometry_header", "GEOMETRY", "", 1, True)
            inputs.addValueInput("L", "Total length", "mm",
                                 adsk.core.ValueInput.createByString("500 mm"))
            inputs.addIntegerSpinnerCommandInput("N", "Number of segments", 1, 200, 1, 40)
            inputs.addValueInput("D0", "Base diameter", "mm",
                                 adsk.core.ValueInput.createByString("60 mm"))
            inputs.addValueInput("D1", "Tip diameter", "mm",
                                 adsk.core.ValueInput.createByString("15 mm"))

            # ---------------- JOINTS ----------------
            inputs.addTextBoxCommandInput("joint_header", "JOINTS & SPINE", "", 1, True)
            inputs.addValueInput("BEND", "Maximum joint bend", "deg",
                                 adsk.core.ValueInput.createByString("12 deg"))
            inputs.addValueInput("ROD_D", "Spine diameter", "mm",
                                 adsk.core.ValueInput.createByString("4 mm"))
            inputs.addValueInput("BORE_CLEAR", "Bore clearance", "mm",
                                 adsk.core.ValueInput.createByString("0.5 mm"))

            # ---------------- TENDONS ----------------
            inputs.addTextBoxCommandInput("tendon_header", "TENDONS", "", 1, True)
            inputs.addIntegerSpinnerCommandInput("N_TENDONS", "Number of tendons", 1, 12, 1, 3)
            inputs.addValueInput("TENDON_D", "Tendon diameter", "mm",
                                 adsk.core.ValueInput.createByString("1.5 mm"))
            inputs.addValueInput("TENDON_R_FRAC", "Tendon radial fraction", "",
                                 adsk.core.ValueInput.createByReal(0.32))
            inputs.addValueInput("PHASE", "First tendon angle", "deg",
                                 adsk.core.ValueInput.createByString("90 deg"))

            # ---------------- SCALLOPS ----------------
            inputs.addTextBoxCommandInput("scallop_header", "CROSS-SECTION", "", 1, True)
            inputs.addIntegerSpinnerCommandInput("SCALLOPS", "Scallops (0 = off, 1 = on)",
                                                 0, 1, 1, 1)
            inputs.addValueInput("SCALLOP_CENTER_FRAC", "Scallop center fraction", "",
                                 adsk.core.ValueInput.createByReal(0.55))
            inputs.addValueInput("SCALLOP_R_FRAC", "Scallop radius fraction", "",
                                 adsk.core.ValueInput.createByReal(0.27))

            # ---------------- ANCHORS ----------------
            inputs.addTextBoxCommandInput("anchor_header", "ANCHORS", "", 1, True)
            inputs.addValueInput("ANCHOR_D", "Anchor diameter (0 = off)", "mm",
                                 adsk.core.ValueInput.createByString("1.9 mm"))
            inputs.addValueInput("ANCHOR_DEPTH", "Anchor depth", "mm",
                                 adsk.core.ValueInput.createByString("1.2 mm"))

            on_execute = SpiRobExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)

            on_destroy = SpiRobDestroyHandler()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)

        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox("Dialog creation failed:\n{}".format(traceback.format_exc()))


def run(context):

    app = adsk.core.Application.get()
    ui = app.userInterface

    try:
        cmd_id = "SpiRobGeneratorCommand"

        old_cmd = ui.commandDefinitions.itemById(cmd_id)
        if old_cmd:
            old_cmd.deleteMe()

        cmd_def = ui.commandDefinitions.addButtonDefinition(
            cmd_id,
            "SpiRob Generator",
            "Configure and generate a SpiRob"
        )

        on_created = SpiRobCommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        cmd_def.execute()

        # Keep the script alive while the dialog is open
        adsk.autoTerminate(False)

    except Exception:
        ui.messageBox("Failed:\n{}".format(traceback.format_exc()))