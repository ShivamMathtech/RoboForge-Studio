import type { Sample } from "../types/robot";

export function TelemetryChart({ samples }: { samples: Sample[] }) {
  const width = 800;
  const height = 150;
  const values = samples.flatMap((point) => [point.value, point.reference]);
  const min = Math.min(-0.05, ...values);
  const max = Math.max(0.05, ...values);
  const span = Math.max(max - min, 0.001);
  const x = (index: number) => (index / Math.max(samples.length - 1, 1)) * width;
  const y = (value: number) => height - ((value - min) / span) * (height - 10) - 5;
  const path = (key: "value" | "reference") =>
    samples.map((point, index) => `${index ? "L" : "M"}${x(index).toFixed(1)},${y(point[key]).toFixed(1)}`).join(" ");

  return (
    <div className="chart-wrap">
      <svg viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Joint tracking telemetry">
        <defs>
          <pattern id="chart-grid" width="50" height="30" patternUnits="userSpaceOnUse">
            <path d="M 50 0 L 0 0 0 30" fill="none" stroke="#1b2b39" strokeWidth="1" />
          </pattern>
          <linearGradient id="area" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0" stopColor="#25dca6" stopOpacity="0.25" />
            <stop offset="1" stopColor="#25dca6" stopOpacity="0" />
          </linearGradient>
        </defs>
        <rect width={width} height={height} fill="url(#chart-grid)" />
        {samples.length > 1 && (
          <>
            <path d={`${path("value")} L${width},${height} L0,${height} Z`} fill="url(#area)" />
            <path d={path("reference")} fill="none" stroke="#698096" strokeWidth="1.5" strokeDasharray="7 5" />
            <path d={path("value")} fill="none" stroke="#25dca6" strokeWidth="2.2" />
          </>
        )}
      </svg>
      <div className="chart-legend"><span><i className="actual" />Actual q₁</span><span><i />Reference</span></div>
    </div>
  );
}

