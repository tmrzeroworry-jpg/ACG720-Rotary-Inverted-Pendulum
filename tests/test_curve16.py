import json, math, struct, subprocess, sys, unittest, shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Mac上位机"))
from trajectory import validate_curve, example_points, Trajectory, packet, Q


class Curve16(unittest.TestCase):
    def test_defaults_have_16_unique_nodes_and_higher_limits(self):
        t = Trajectory()
        c = t.curve
        self.assertEqual(len(c["points"]), 16)
        self.assertEqual(c["vmax"], 180)
        self.assertTrue(350 < c["amax"] <= 360)
        self.assertEqual(c["points"][0], [0, 0])
        self.assertEqual(c["points"][-1], [15, 0])

    def test_accepted_high_curve_and_packet(self):
        c = validate_curve(
            dict(
                points=[[0, 0], [1, 350], [2, 0], [3, -350], [4, 0]], vmax=360, amax=720
            )
        )
        self.assertEqual(c["acc_step"], 136)
        b = packet(2, 1, 123, 360, c["acc_step"])
        self.assertEqual(struct.unpack(">i", b[12:16])[0], 68157)
        self.assertEqual(struct.unpack(">H", b[16:18])[0], 136)
        self.assertAlmostEqual(c["amax"], 136 * 1000 / Q)

    def test_limits_nodes_and_duplicate_rejection(self):
        for data in [
            dict(points=example_points(), vmax=361),
            dict(points=example_points(), amax=721),
            dict(points=[[i, 0] for i in range(17)]),
            dict(points=[[0, 0], [1, 40], [1, 50], [2, 0]]),
        ]:
            with self.assertRaises(ValueError):
                validate_curve(data)

    def test_quantized_peak_rejected(self):
        with self.assertRaises(ValueError):
            validate_curve(
                dict(points=[[0, 0], [1, 360], [2, -360], [3, 0]], vmax=360, amax=720)
            )

    def test_slow_deceleration_budget_is_sufficient(self):
        t = Trajectory()
        t.curve = validate_curve(
            dict(points=[[0, 0], [50, 250], [100, 0]], vmax=360, amax=6)
        )
        t.phase = "running"
        t.stop()
        remaining = t.deadline - t.clock()
        self.assertGreater(remaining, 360 / t.curve["amax"])
        self.assertLessEqual(remaining, 70)

    def test_python_and_browser_validation_match(self):
        cases = [
            dict(points=example_points(), vmax=180, amax=360),
            dict(points=[[0, 0], [1, 350], [2, 0]], vmax=360, amax=720),
            dict(points=[[0, 0], [1, 360], [2, -360], [3, 0]], vmax=360, amax=720),
            dict(points=[[0, 0], [1, 10], [1, 20], [2, 0]]),
            dict(points=[[i, 0] for i in range(17)]),
        ]
        code = (
            "const m=require('./Mac上位机/web/trajectory_math.js');const cases="
            + json.dumps(cases)
            + ";process.stdout.write(JSON.stringify(cases.map(x=>{try{return {ok:true,data:m.validate(x)}}catch(e){return {ok:false}}})));"
        )
        results = json.loads(
            subprocess.check_output(
                [shutil.which("node") or "node", "-e", code], cwd=ROOT, text=True
            )
        )
        for c, r in zip(cases, results):
            try:
                v = validate_curve(c)
            except ValueError:
                self.assertFalse(r["ok"])
            else:
                self.assertTrue(r["ok"])
                self.assertEqual(v, r["data"])

    def test_browser_example_equals_python(self):
        r = json.loads(
            subprocess.check_output(
                [
                    shutil.which("node") or "node",
                    "-e",
                    "process.stdout.write(JSON.stringify(require('./Mac上位机/web/trajectory_math.js').example(15)))",
                ],
                cwd=ROOT,
                text=True,
            )
        )
        self.assertEqual(r, example_points())


if __name__ == "__main__":
    unittest.main()
